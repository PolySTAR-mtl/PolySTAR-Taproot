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

    in {
      devShell = pkgs.mkShell {
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
        ];

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
