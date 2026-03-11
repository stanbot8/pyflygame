"""CLI entry point for flygame (nmfly command)."""

import argparse
import logging


def main() -> None:
    parser = argparse.ArgumentParser(
        description="flygame: high-performance flygym/NeuroMechFly on MuJoCo"
    )
    parser.add_argument("--dt", type=float, default=0.0002,
                        help="Physics timestep in seconds (default: 0.0002)")
    parser.add_argument("--standalone", action="store_true",
                        help="Run without FWMC connection (walk forward)")

    # FWMC connection
    parser.add_argument("--host", default="127.0.0.1",
                        help="FWMC TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9100,
                        help="FWMC TCP port (default: 9100)")

    # Backend selection
    parser.add_argument("--backend", choices=["flygym", "builtin", "wasd", "launch"],
                        default="flygym",
                        help="Body sim backend (default: flygym)")
    parser.add_argument("--brain-exe", default="",
                        help="Path to fwmc-viewer.exe (for --backend launch)")

    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    if args.backend == "flygym":
        from .flygym_bridge import run_bridge
        run_bridge(
            host=args.host,
            port=args.port,
            timestep=args.dt,
            standalone=args.standalone,
        )
    elif args.backend == "builtin":
        from .bridge import Bridge, BridgeConfig
        cfg = BridgeConfig(dt=args.dt, render=True)
        if args.standalone:
            bridge = Bridge(cfg)
            bridge.run()
        else:
            from .adapters.fwmc import make_fwmc_controller
            bridge = Bridge(cfg)
            bridge.run(make_fwmc_controller(args.host, args.port))
    elif args.backend == "wasd":
        from .flygym_wasd import run_wasd
        run_wasd(timestep=args.dt)
    elif args.backend == "launch":
        from .launcher import launch
        launch(
            brain_exe=args.brain_exe,
            host=args.host,
            port=args.port,
            dt=args.dt,
        )


if __name__ == "__main__":
    main()
