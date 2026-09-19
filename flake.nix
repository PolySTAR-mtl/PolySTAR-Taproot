{
  description = "PolyStar's Nix Shell";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-23.05";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
      };

      python = pkgs.python3;
      pythonPackages = python.pkgs;

      lbuild = pythonPackages.buildPythonPackage rec {
        pname = "lbuild";
        version = "1.20.0";

        src = pythonPackages.fetchPypi {
          inherit pname version;
          sha256 = "sha256-nHBniVpEzuQ9EXeFHxHqrI4kPfsF94AhmRdbb//TD88=";
        };

        propagatedBuildInputs = with pythonPackages; [
          colorful
          anytree
          jinja2
          lxml
          gitpython
        ];

        doCheck = false;
      };

      mkRobotCommand = { name, action, robot }:
        pkgs.writeShellScriptBin name ''
          set -euo pipefail

          if [ -n "''${POLYSTAR_PROJECT_DIR:-}" ]; then
            project_dir="$POLYSTAR_PROJECT_DIR"
          elif [ -f "$PWD/SConstruct" ]; then
            project_dir="$PWD"
          else
            project_dir="$PWD/PolySTAR-Taproot-project"
          fi

          if [ ! -f "$project_dir/SConstruct" ]; then
            echo "Could not find PolySTAR-Taproot-project/SConstruct." >&2
            echo "Run this command from the repository root or set POLYSTAR_PROJECT_DIR." >&2
            exit 1
          fi

          cd "$project_dir"

          flake_dir="''${POLYSTAR_FLAKE_DIR:-$(dirname "$project_dir")}"

          exec nix develop "$flake_dir" -c scons ${action} robot=${robot} "$@"
        '';

      robotCommands = {
        build-standard = mkRobotCommand {
          name = "build-standard";
          action = "build";
          robot = "TARGET_STANDARD";
        };
        run-standard = mkRobotCommand {
          name = "run-standard";
          action = "run";
          robot = "TARGET_STANDARD";
        };
        build-sentry = mkRobotCommand {
          name = "build-sentry";
          action = "build";
          robot = "TARGET_SENTRY";
        };
        run-sentry = mkRobotCommand {
          name = "run-sentry";
          action = "run";
          robot = "TARGET_SENTRY";
        };
        build-hero = mkRobotCommand {
          name = "build-hero";
          action = "build";
          robot = "TARGET_HERO";
        };
        run-hero = mkRobotCommand {
          name = "run-hero";
          action = "run";
          robot = "TARGET_HERO";
        };
      };

    in {
      apps = builtins.mapAttrs (_: command: {
        type = "app";
        program = "${command}/bin/${command.name}";
      }) robotCommands;

      devShells.default = pkgs.mkShell {
        nativeBuildInputs = with pkgs; [
          pkg-config
          gcc-arm-embedded-10
          gcc10
          binutils
          scons
          gnumake
          automake
          git
          doxygen
          gtest
          bear
          glibc
          clang-tools
        ] ++ builtins.attrValues robotCommands;

        buildInputs = with pkgs; [
          python
          pythonPackages.pip
          pythonPackages.setuptools
          pythonPackages.virtualenv
          pythonPackages.pyelftools
          # pythonPackages.standard-telnetlib
          pythonPackages.jinja2
          lbuild
          stlink
          openocd
          boost
          libusb1
        ];

      shellHook = ''
        python --version
        g++ --version
      '';
    };
  });
}
