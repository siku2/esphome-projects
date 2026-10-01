{
  description = "ESPHome components, packages and device configurations";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      treefmt-nix,
    }:
    let
      inherit (nixpkgs) lib;

      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];

      pythonFiles = lib.fileset.fileFilter (file: file.hasExt "py");

      simulatorSrc = lib.fileset.toSource {
        root = ./simulator;
        fileset = lib.fileset.unions [
          ./simulator/pyproject.toml
          (pythonFiles ./simulator)
        ];
      };

      perSystem = lib.genAttrs systems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          python = pkgs.python3;
          # The nixpkgs sdl2-config has an empty includedir and prints -I/SDL2.
          sdl2 = pkgs.SDL2.overrideAttrs (old: {
            postFixup = (old.postFixup or "") + ''
              substituteInPlace "$dev/bin/sdl2-config" \
                --replace-fail "in /SDL2 " 'in ''${includedir}/SDL2 '
            '';
          });
        in
        {
          inherit pkgs sdl2;
          pythonDev = python.withPackages (ps: [
            ps.aioesphomeapi
            ps.mypy
            ps.pytest
            ps.pyyaml
            ps.types-pyyaml
          ]);
          esphomeSim = python.pkgs.buildPythonApplication {
            pname = "esphome-sim";
            version = "0.1.0";
            pyproject = true;
            src = simulatorSrc;
            build-system = [ python.pkgs.hatchling ];
            dependencies = [
              python.pkgs.aioesphomeapi
              python.pkgs.pyyaml
              python.pkgs.tkinter
            ];
            nativeCheckInputs = [ python.pkgs.pytestCheckHook ];
            pythonImportsCheck = [ "esphome_sim" ];
            # `esphome-sim run` compiles the host build.
            makeWrapperArgs = [
              "--prefix"
              "PATH"
              ":"
              (lib.makeBinPath [
                pkgs.esphome
                sdl2
                sdl2.dev
                pkgs.openssl
                pkgs.openssl.dev
                pkgs.stdenv.cc
              ])
              "--prefix"
              "CPATH"
              ":"
              "${pkgs.openssl.dev}/include"
              "--prefix"
              "LIBRARY_PATH"
              ":"
              "${lib.getLib pkgs.openssl}/lib"
            ];
            meta.mainProgram = "esphome-sim";
          };
          treefmt = treefmt-nix.lib.evalModule pkgs ./treefmt.nix;
          validateConfigs = pkgs.writeShellApplication {
            name = "validate-configs";
            runtimeInputs = [
              pkgs.esphome
              pkgs.git
              sdl2.dev
            ];
            text = ''
              cd "$(git rev-parse --show-toplevel)"
              if [ "$#" -eq 0 ]; then set -- tests/*.yaml; fi
              exec esphome config "$@"
            '';
          };
        }
      );

      forSystems = f: lib.mapAttrs (_: f) perSystem;
    in
    {
      formatter = forSystems ({ treefmt, ... }: treefmt.config.build.wrapper);

      packages = forSystems (
        { esphomeSim, ... }:
        {
          esphome-sim = esphomeSim;
        }
      );

      checks = forSystems (
        {
          pkgs,
          treefmt,
          pythonDev,
          esphomeSim,
          ...
        }:
        {
          formatting = treefmt.config.build.check self;

          # treefmt runs `ruff check --fix`, which reports what it cannot fix as
          # a formatting failure.
          lint-python =
            pkgs.runCommand "check-lint-python"
              {
                nativeBuildInputs = [ pkgs.ruff ];
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./ruff.toml
                    (pythonFiles ./components)
                    (pythonFiles ./projects)
                    (pythonFiles ./simulator)
                  ];
                };
              }
              ''
                ruff check --no-cache "$src"
                touch "$out"
              '';

          typecheck-python =
            pkgs.runCommand "check-typecheck-python"
              {
                nativeBuildInputs = [ pythonDev ];
                src = simulatorSrc;
              }
              ''
                cd "$src"
                mypy --strict --cache-dir "$TMPDIR/mypy" esphome_sim tests
                touch "$out"
              '';

          esphome-sim = esphomeSim;

          test-grill-cook =
            pkgs.runCommand "check-test-grill-cook"
              {
                nativeBuildInputs = [ pkgs.stdenv.cc ];
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./components/grill_cook/cook_model.h
                    ./components/grill_cook/tests/cook_model_test.cpp
                  ];
                };
              }
              ''
                g++ -std=c++17 -Wall -Wextra -Werror \
                  -o cook_model_test "$src/components/grill_cook/tests/cook_model_test.cpp"
                ./cook_model_test
                touch "$out"
              '';
        }
      );

      apps = forSystems (
        { validateConfigs, esphomeSim, ... }:
        {
          validate-configs = {
            type = "app";
            program = lib.getExe validateConfigs;
            meta.description = "Run `esphome config` on every device configuration";
          };
          sim = {
            type = "app";
            program = lib.getExe esphomeSim;
            meta.description = "Run and drive ESPHome host simulations";
          };
        }
      );

      devShells = forSystems (
        {
          pkgs,
          sdl2,
          treefmt,
          validateConfigs,
          pythonDev,
          esphomeSim,
          ...
        }:
        {
          default = pkgs.mkShell {
            packages = [
              pkgs.esphome
              sdl2
              sdl2.dev
              pkgs.openssl
              pkgs.openssl.dev
              pkgs.clang-tools
              pkgs.ruff
              pythonDev
              treefmt.config.build.wrapper
              validateConfigs
              esphomeSim
            ];
          };
        }
      );
    };
}
