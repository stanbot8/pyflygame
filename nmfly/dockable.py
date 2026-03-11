"""Dockable window manager for Win32.

Manages a host window with docked panels. Panels start as small
borderless thumbnails pinned to the host's client area. A small
pop-out button in the corner expands them to full size. The panel
remains interactive while docked (you can orbit/click inside it).

Usage:
    from nmfly.dockable import DockManager

    mgr = DockManager()
    mgr.set_host(title="MuJoCo")
    mgr.add_panel("brain", title="FWMC Brain Viewer",
                   anchor="bottom-right", size=(0.35, 0.4))
    mgr.discover()
    mgr.apply()
    mgr.start_follow()
"""

import ctypes
import ctypes.wintypes
import enum
import logging
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger("nmfly.dock")

# ── Win32 constants ──────────────────────────────────────────────────────

GWL_STYLE = -16
GWL_EXSTYLE = -20
GWL_WNDPROC = -4

WS_POPUP = 0x80000000
WS_VISIBLE = 0x10000000
WS_CAPTION = 0x00C00000
WS_THICKFRAME = 0x00040000
WS_MINIMIZEBOX = 0x00020000
WS_MAXIMIZEBOX = 0x00010000
WS_SYSMENU = 0x00080000
WS_EX_CLIENTEDGE = 0x00000200
WS_EX_DLGMODALFRAME = 0x00000001
WS_EX_TOPMOST = 0x00000008
WS_EX_TOOLWINDOW = 0x00000080
WS_EX_LAYERED = 0x00080000

SWP_NOACTIVATE = 0x0010
SWP_NOZORDER = 0x0004
SWP_FRAMECHANGED = 0x0020
SWP_NOMOVE = 0x0002
SWP_NOSIZE = 0x0001
SWP_SHOWWINDOW = 0x0040

HWND_TOPMOST = ctypes.wintypes.HWND(-1)
HWND_NOTOPMOST = ctypes.wintypes.HWND(-2)

WM_CLOSE = 0x0010
WM_DESTROY = 0x0002
WM_LBUTTONDOWN = 0x0201
WM_PAINT = 0x000F
WM_CREATE = 0x0001
WM_KEYDOWN = 0x0100
WM_KEYUP = 0x0101

WH_KEYBOARD_LL = 13
HC_ACTION = 0

# Virtual key codes for WASD + space + R
VK_WASD = {0x57, 0x41, 0x53, 0x44, 0x20, 0x52}  # W, A, S, D, Space, R

LWA_ALPHA = 0x02
COLOR_BTNFACE = 15

BORDER_BITS = (WS_CAPTION | WS_THICKFRAME | WS_MINIMIZEBOX |
               WS_MAXIMIZEBOX | WS_SYSMENU)
EX_BORDER_BITS = WS_EX_CLIENTEDGE | WS_EX_DLGMODALFRAME

# ── Win32 API ────────────────────────────────────────────────────────────

user32 = ctypes.windll.user32
gdi32 = ctypes.windll.gdi32
kernel32 = ctypes.windll.kernel32

EnumWindows = user32.EnumWindows
GetWindowTextW = user32.GetWindowTextW
GetWindowThreadProcessId = user32.GetWindowThreadProcessId
SetWindowPos = user32.SetWindowPos
IsWindowVisible = user32.IsWindowVisible
SystemParametersInfoW = user32.SystemParametersInfoW
SetForegroundWindow = user32.SetForegroundWindow
GetWindowLongW = user32.GetWindowLongW
SetWindowLongW = user32.SetWindowLongW
GetWindowRect = user32.GetWindowRect
GetClientRect = user32.GetClientRect
ClientToScreen = user32.ClientToScreen
IsWindow = user32.IsWindow
PostMessageW = user32.PostMessageW
ShowWindow = user32.ShowWindow
GetForegroundWindow = user32.GetForegroundWindow
DestroyWindow = user32.DestroyWindow
SetLayeredWindowAttributes = user32.SetLayeredWindowAttributes
InvalidateRect = user32.InvalidateRect

# GDI for painting the button
BeginPaint = user32.BeginPaint
EndPaint = user32.EndPaint
FillRect = user32.FillRect
DrawTextW = user32.DrawTextW
SetBkMode = gdi32.SetBkMode
SetTextColor = gdi32.SetTextColor
CreateFontW = gdi32.CreateFontW
SelectObject = gdi32.SelectObject
DeleteObject = gdi32.DeleteObject
CreateSolidBrush = gdi32.CreateSolidBrush

ENUMWINDOWSPROC = ctypes.WINFUNCTYPE(
    ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM
)

# Low-level keyboard hook callback type
HOOKPROC = ctypes.WINFUNCTYPE(
    ctypes.c_ssize_t, ctypes.c_int, ctypes.wintypes.WPARAM, ctypes.wintypes.LPARAM
)

SetWindowsHookExW = user32.SetWindowsHookExW
SetWindowsHookExW.argtypes = [ctypes.c_int, HOOKPROC, ctypes.wintypes.HINSTANCE, ctypes.wintypes.DWORD]
SetWindowsHookExW.restype = ctypes.c_void_p

UnhookWindowsHookEx = user32.UnhookWindowsHookEx
CallNextHookEx = user32.CallNextHookEx
CallNextHookEx.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.wintypes.WPARAM, ctypes.wintypes.LPARAM]
CallNextHookEx.restype = ctypes.c_ssize_t

SendMessageW = user32.SendMessageW

class KBDLLHOOKSTRUCT(ctypes.Structure):
    _fields_ = [
        ("vkCode", ctypes.wintypes.DWORD),
        ("scanCode", ctypes.wintypes.DWORD),
        ("flags", ctypes.wintypes.DWORD),
        ("time", ctypes.wintypes.DWORD),
        ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong)),
    ]

WNDPROC = ctypes.WINFUNCTYPE(
    ctypes.c_ssize_t, ctypes.wintypes.HWND, ctypes.c_uint,
    ctypes.wintypes.WPARAM, ctypes.wintypes.LPARAM
)

# DefWindowProcW needs explicit arg/res types for 64-bit
user32.DefWindowProcW.argtypes = [
    ctypes.wintypes.HWND, ctypes.c_uint,
    ctypes.wintypes.WPARAM, ctypes.wintypes.LPARAM,
]
user32.DefWindowProcW.restype = ctypes.c_ssize_t

# SetWindowLongPtrW for 64-bit pointer values
try:
    SetWindowLongPtrW = user32.SetWindowLongPtrW
except AttributeError:
    SetWindowLongPtrW = user32.SetWindowLongW
SetWindowLongPtrW.argtypes = [ctypes.wintypes.HWND, ctypes.c_int, ctypes.c_ssize_t]
SetWindowLongPtrW.restype = ctypes.c_ssize_t

# PAINTSTRUCT
class PAINTSTRUCT(ctypes.Structure):
    _fields_ = [
        ("hdc", ctypes.wintypes.HDC),
        ("fErase", ctypes.wintypes.BOOL),
        ("rcPaint", ctypes.wintypes.RECT),
        ("fRestore", ctypes.wintypes.BOOL),
        ("fIncUpdate", ctypes.wintypes.BOOL),
        ("rgbReserved", ctypes.c_byte * 32),
    ]


# ── Win32 helpers ────────────────────────────────────────────────────────

def get_work_area() -> tuple[int, int, int, int]:
    r = ctypes.wintypes.RECT()
    SystemParametersInfoW(0x0030, 0, ctypes.byref(r), 0)
    return r.left, r.top, r.right - r.left, r.bottom - r.top


def get_rect(hwnd: int) -> tuple[int, int, int, int]:
    r = ctypes.wintypes.RECT()
    GetWindowRect(hwnd, ctypes.byref(r))
    return r.left, r.top, r.right - r.left, r.bottom - r.top


def get_client_rect_screen(hwnd: int) -> tuple[int, int, int, int]:
    cr = ctypes.wintypes.RECT()
    GetClientRect(hwnd, ctypes.byref(cr))
    pt = ctypes.wintypes.POINT(0, 0)
    ClientToScreen(hwnd, ctypes.byref(pt))
    return pt.x, pt.y, cr.right, cr.bottom


def find_window(title_substr: str, timeout: float = 15.0) -> int:
    result = [0]
    deadline = time.monotonic() + timeout
    buf = ctypes.create_unicode_buffer(256)

    while time.monotonic() < deadline:
        def callback(hwnd, _lp):
            if not IsWindowVisible(hwnd):
                return True
            GetWindowTextW(hwnd, buf, 256)
            if title_substr in buf.value:
                result[0] = hwnd
                return False
            return True

        EnumWindows(ENUMWINDOWSPROC(callback), 0)
        if result[0]:
            return result[0]
        time.sleep(0.3)

    return 0


def strip_border(hwnd: int) -> tuple[int, int]:
    style = GetWindowLongW(hwnd, GWL_STYLE)
    ex = GetWindowLongW(hwnd, GWL_EXSTYLE)
    SetWindowLongW(hwnd, GWL_STYLE, style & ~BORDER_BITS)
    SetWindowLongW(hwnd, GWL_EXSTYLE, ex & ~EX_BORDER_BITS)
    SetWindowPos(hwnd, 0, 0, 0, 0, 0,
                 SWP_FRAMECHANGED | SWP_NOACTIVATE | SWP_NOZORDER |
                 SWP_NOMOVE | SWP_NOSIZE)
    return style, ex


def restore_border(hwnd: int, style: int, ex_style: int):
    SetWindowLongW(hwnd, GWL_STYLE, style)
    SetWindowLongW(hwnd, GWL_EXSTYLE, ex_style)
    SetWindowPos(hwnd, 0, 0, 0, 0, 0,
                 SWP_FRAMECHANGED | SWP_NOACTIVATE | SWP_NOZORDER |
                 SWP_NOMOVE | SWP_NOSIZE)


# ── Anchor positions ─────────────────────────────────────────────────────

class Anchor(enum.Enum):
    BOTTOM_RIGHT = "bottom-right"
    BOTTOM_LEFT = "bottom-left"
    TOP_RIGHT = "top-right"
    TOP_LEFT = "top-left"


def _anchor_rect(anchor: Anchor, client: tuple[int, int, int, int],
                 frac_w: float, frac_h: float) -> tuple[int, int, int, int]:
    cx, cy, cw, ch = client
    pw = int(cw * frac_w)
    ph = int(ch * frac_h)

    if anchor == Anchor.BOTTOM_RIGHT:
        return (cx + cw - pw, cy + ch - ph, pw, ph)
    elif anchor == Anchor.BOTTOM_LEFT:
        return (cx, cy + ch - ph, pw, ph)
    elif anchor == Anchor.TOP_RIGHT:
        return (cx + cw - pw, cy, pw, ph)
    elif anchor == Anchor.TOP_LEFT:
        return (cx, cy, pw, ph)
    return (cx + cw - pw, cy + ch - ph, pw, ph)


# ── Pop-out button overlay ───────────────────────────────────────────────

# Button size
BTN_W = 40
BTN_H = 28

# Keep references alive so GC doesn't collect the callback
_wndproc_refs: list = []
_registered_class = False


def _register_btn_class():
    """Register a Win32 window class for the pop-out button."""
    global _registered_class
    if _registered_class:
        return

    WNDCLASSEXW = type("WNDCLASSEXW", (ctypes.Structure,), {
        "_fields_": [
            ("cbSize", ctypes.c_uint),
            ("style", ctypes.c_uint),
            ("lpfnWndProc", WNDPROC),
            ("cbClsExtra", ctypes.c_int),
            ("cbWndExtra", ctypes.c_int),
            ("hInstance", ctypes.wintypes.HINSTANCE),
            ("hIcon", ctypes.wintypes.HICON),
            ("hCursor", ctypes.wintypes.HICON),
            ("hbrBackground", ctypes.wintypes.HBRUSH),
            ("lpszMenuName", ctypes.wintypes.LPCWSTR),
            ("lpszClassName", ctypes.wintypes.LPCWSTR),
            ("hIconSm", ctypes.wintypes.HICON),
        ]
    })

    def default_proc(hwnd, msg, wp, lp):
        return user32.DefWindowProcW(hwnd, msg, wp, lp)

    proc = WNDPROC(default_proc)
    _wndproc_refs.append(proc)

    wc = WNDCLASSEXW()
    wc.cbSize = ctypes.sizeof(WNDCLASSEXW)
    wc.style = 0
    wc.lpfnWndProc = proc
    wc.hInstance = kernel32.GetModuleHandleW(None)
    wc.hbrBackground = gdi32.GetStockObject(0)  # BLACK_BRUSH
    wc.lpszClassName = "NmflyDockBtn"
    wc.hCursor = user32.LoadCursorW(None, ctypes.wintypes.LPCWSTR(32649))  # IDC_HAND

    user32.RegisterClassExW(ctypes.byref(wc))
    _registered_class = True


def _create_btn(x: int, y: int, label: str, on_click) -> int:
    """Create a small overlay button at (x, y) screen coords.

    Returns the window handle. on_click is called when button is clicked.
    """
    _register_btn_class()

    def wndproc(hwnd, msg, wp, lp):
        if msg == WM_LBUTTONDOWN:
            on_click()
            return 0
        if msg == WM_PAINT:
            ps = PAINTSTRUCT()
            hdc = BeginPaint(hwnd, ctypes.byref(ps))

            # Bright blue background so it's clearly visible
            brush = CreateSolidBrush(0x00CC6600)  # BGR: orange-ish blue
            rc = ctypes.wintypes.RECT(0, 0, BTN_W, BTN_H)
            FillRect(hdc, ctypes.byref(rc), brush)
            DeleteObject(brush)

            # Draw label text (use ASCII chars that always render)
            font = CreateFontW(
                18, 0, 0, 0, 700, 0, 0, 0, 0, 0, 0, 0, 0, "Consolas")
            old_font = SelectObject(hdc, font)
            SetBkMode(hdc, 1)  # TRANSPARENT
            SetTextColor(hdc, 0x00FFFFFF)  # white
            text = ctypes.create_unicode_buffer(label)
            DrawTextW(hdc, text, -1, ctypes.byref(rc), 0x0025)  # CENTER|VCENTER|SINGLELINE
            SelectObject(hdc, old_font)
            DeleteObject(font)

            EndPaint(hwnd, ctypes.byref(ps))
            return 0
        return user32.DefWindowProcW(hwnd, msg, wp, lp)

    proc = WNDPROC(wndproc)
    _wndproc_refs.append(proc)

    style = WS_POPUP | WS_VISIBLE
    ex_style = WS_EX_TOPMOST | WS_EX_TOOLWINDOW

    hwnd = user32.CreateWindowExW(
        ex_style,
        "NmflyDockBtn",
        None,
        style,
        x, y, BTN_W, BTN_H,
        0, 0,
        kernel32.GetModuleHandleW(None),
        0,
    )

    # Override wndproc
    SetWindowLongPtrW(hwnd, GWL_WNDPROC, ctypes.cast(proc, ctypes.c_void_p).value)
    InvalidateRect(hwnd, None, True)

    return hwnd


# ── Panel ────────────────────────────────────────────────────────────────

@dataclass
class Panel:
    """A dockable panel window."""
    name: str
    hwnd: int = 0
    title: str = ""
    anchor: Anchor = Anchor.BOTTOM_RIGHT
    frac_w: float = 0.35
    frac_h: float = 0.4
    docked: bool = True
    # internal
    _saved_style: int = 0
    _saved_ex: int = 0
    _docked_rect: tuple[int, int, int, int] = (0, 0, 0, 0)
    _undocked_rect: tuple[int, int, int, int] = (0, 0, 0, 0)
    _btn_hwnd: int = 0


# ── DockManager ──────────────────────────────────────────────────────────

class DockManager:
    """Manages a host window with pop-out panels.

    The host window keeps its native chrome and input. Panels start as
    borderless thumbnails docked to corners of the host. A small button
    in the upper-right corner pops them out to full size. The panel
    remains fully interactive while docked (orbit, click, etc.).
    """

    def __init__(self):
        self._host_hwnd: int = 0
        self._host_title: str = ""
        self._panels: list[Panel] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._poll_interval: float = 0.15
        self._last_host_rect: tuple[int, int, int, int] | None = None

    def set_host(self, hwnd: int = 0, title: str = ""):
        self._host_hwnd = hwnd
        self._host_title = title

    def add_panel(self, name: str, hwnd: int = 0, title: str = "",
                  anchor: str = "bottom-right",
                  size: tuple[float, float] = (0.35, 0.4)):
        self._panels.append(Panel(
            name=name, hwnd=hwnd, title=title,
            anchor=Anchor(anchor),
            frac_w=size[0], frac_h=size[1],
        ))

    def discover(self, timeout: float = 30.0):
        if self._host_title and not self._host_hwnd:
            self._host_hwnd = find_window(self._host_title, timeout=timeout)
            if self._host_hwnd:
                log.info("Found host '%s' (hwnd=%d)",
                         self._host_title, self._host_hwnd)

        for p in self._panels:
            if p.title and not p.hwnd:
                p.hwnd = find_window(p.title, timeout=timeout)
                if p.hwnd:
                    log.info("Found panel '%s' (hwnd=%d)", p.name, p.hwnd)

    def apply(self):
        """Dock all panels and create pop-out buttons."""
        if not self._host_hwnd:
            return

        self._last_host_rect = get_rect(self._host_hwnd)
        client = get_client_rect_screen(self._host_hwnd)

        for p in self._panels:
            if not p.hwnd:
                continue

            p._docked_rect = _anchor_rect(p.anchor, client, p.frac_w, p.frac_h)

            p.docked = True
            x, y, w, h = p._docked_rect
            # Position panel above host without modifying window style
            SetWindowPos(p.hwnd, ctypes.wintypes.HWND(0), x, y, w, h,
                         SWP_NOACTIVATE)

            log.debug("Panel '%s' docked: %dx%d at (%d,%d)", p.name, w, h, x, y)

        SetForegroundWindow(self._host_hwnd)
        # Bring host to front, then panels above it
        for p in self._panels:
            if p.hwnd and p.docked:
                SetWindowPos(p.hwnd, ctypes.wintypes.HWND(0),
                             0, 0, 0, 0,
                             SWP_NOACTIVATE | SWP_NOMOVE | SWP_NOSIZE)
        log.info("Layout applied: %d panels docked", len(self._panels))

    def _on_pop_click(self, panel: Panel):
        """Called when the pop-out/dock button is clicked."""
        if panel.docked:
            self.pop_out(panel)
        else:
            self.dock_back(panel)

    def pop_out(self, panel: Panel):
        if not panel.hwnd or not panel.docked:
            return

        panel.docked = False
        restore_border(panel.hwnd, panel._saved_style, panel._saved_ex)

        x, y, w, h = panel._undocked_rect
        SetWindowPos(panel.hwnd, HWND_NOTOPMOST, x, y, w, h, 0)
        SetForegroundWindow(panel.hwnd)

        # Move button to upper-right of undocked window, change to dock-back icon
        if panel._btn_hwnd:
            DestroyWindow(panel._btn_hwnd)
            btn_x = x + w - BTN_W - 8
            btn_y = y + 4
            panel._btn_hwnd = _create_btn(btn_x, btn_y, "[v]",
                                           lambda p=panel: self._on_pop_click(p))

        log.info("Panel '%s' popped out: %dx%d at (%d,%d)",
                 panel.name, w, h, x, y)

    def dock_back(self, panel: Panel):
        if not panel.hwnd or panel.docked:
            return

        panel.docked = True
        panel._undocked_rect = get_rect(panel.hwnd)

        client = get_client_rect_screen(self._host_hwnd)
        panel._docked_rect = _anchor_rect(
            panel.anchor, client, panel.frac_w, panel.frac_h)

        x, y, w, h = panel._docked_rect
        SetWindowPos(panel.hwnd, ctypes.wintypes.HWND(0), x, y, w, h, SWP_NOACTIVATE)

        # Move button to upper-right of docked panel, change to pop-out icon
        if panel._btn_hwnd:
            DestroyWindow(panel._btn_hwnd)
            btn_x = x + w - BTN_W - 4
            btn_y = y + 4
            panel._btn_hwnd = _create_btn(btn_x, btn_y, "[^]",
                                           lambda p=panel: self._on_pop_click(p))

        SetForegroundWindow(panel.hwnd)
        log.info("Panel '%s' docked back: %dx%d at (%d,%d)",
                 panel.name, w, h, x, y)

    def _subclass_host(self):
        """Subclass MuJoCo's GLFW window to eat WASD before GLFW sees them.

        This blocks MuJoCo keyboard shortcuts (camera presets, pause, etc.)
        for WASD keys while letting GetAsyncKeyState in the brain viewer
        still read the physical key state.
        """
        if not self._host_hwnd:
            return

        # Get the original WNDPROC so we can call it for non-WASD keys
        try:
            GetWindowLongPtrW = user32.GetWindowLongPtrW
        except AttributeError:
            GetWindowLongPtrW = user32.GetWindowLongW
        GetWindowLongPtrW.argtypes = [ctypes.wintypes.HWND, ctypes.c_int]
        GetWindowLongPtrW.restype = ctypes.c_ssize_t

        self._orig_wndproc = GetWindowLongPtrW(self._host_hwnd, GWL_WNDPROC)

        # CallWindowProcW to chain to original
        user32.CallWindowProcW.argtypes = [
            ctypes.c_ssize_t, ctypes.wintypes.HWND, ctypes.c_uint,
            ctypes.wintypes.WPARAM, ctypes.wintypes.LPARAM,
        ]
        user32.CallWindowProcW.restype = ctypes.c_ssize_t

        orig = self._orig_wndproc

        def subclass_proc(hwnd, msg, wp, lp):
            # Eat WASD key messages so GLFW/MuJoCo never sees them
            if msg in (WM_KEYDOWN, WM_KEYUP) and wp in VK_WASD:
                return 0
            return user32.CallWindowProcW(orig, hwnd, msg, wp, lp)

        self._subclass_proc_ref = WNDPROC(subclass_proc)
        SetWindowLongPtrW(
            self._host_hwnd, GWL_WNDPROC,
            ctypes.cast(self._subclass_proc_ref, ctypes.c_void_p).value,
        )
        log.info("Keyboard hook installed (WASD \u2192 brain viewer)")

    def _unsubclass_host(self):
        if getattr(self, "_orig_wndproc", None) and self._host_hwnd:
            if IsWindow(self._host_hwnd):
                SetWindowLongPtrW(
                    self._host_hwnd, GWL_WNDPROC, self._orig_wndproc)
            self._orig_wndproc = None

    def run_follow(self):
        """Keep docked panels synced and process button messages."""
        if not self._host_hwnd:
            return

        self._subclass_host()

        while not self._stop.is_set():
            # Pump messages for our button windows
            msg = ctypes.wintypes.MSG()
            while user32.PeekMessageW(ctypes.byref(msg), 0, 0, 0, 1):  # PM_REMOVE
                user32.TranslateMessage(ctypes.byref(msg))
                user32.DispatchMessageW(ctypes.byref(msg))

            # Host closed: close all panels
            if not IsWindow(self._host_hwnd):
                log.info("Host closed, closing panels")
                self._unsubclass_host()
                for p in self._panels:
                    if p.hwnd and IsWindow(p.hwnd):
                        PostMessageW(p.hwnd, WM_CLOSE, 0, 0)
                return

            # Panel closed
            for p in self._panels:
                if p.hwnd and not IsWindow(p.hwnd):
                    log.info("Panel '%s' closed", p.name)
                    self._unsubclass_host()
                    return

            # Reflow docked panels if host moved/resized
            cur = get_rect(self._host_hwnd)
            host_is_fg = (GetForegroundWindow() == self._host_hwnd)
            moved = (cur != self._last_host_rect)

            if moved or host_is_fg:
                client = get_client_rect_screen(self._host_hwnd)
                for p in self._panels:
                    if not p.docked or not p.hwnd:
                        continue
                    if moved:
                        p._docked_rect = _anchor_rect(
                            p.anchor, client, p.frac_w, p.frac_h)
                    x, y, w, h = p._docked_rect
                    # Keep panel above host, not globally topmost
                    SetWindowPos(p.hwnd, ctypes.wintypes.HWND(0),
                                 x, y, w, h, SWP_NOACTIVATE)
                self._last_host_rect = cur

            self._stop.wait(self._poll_interval)

    def start_follow(self) -> threading.Thread:
        self._stop.clear()
        t = threading.Thread(target=self.run_follow, daemon=True,
                             name="dock-follow")
        t.start()
        self._thread = t
        return t

    def stop(self):
        self._stop.set()
        self._unsubclass_host()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        pass  # panels close with host
