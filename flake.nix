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
          pipenv
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
