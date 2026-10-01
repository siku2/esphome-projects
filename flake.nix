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

      perSystem = lib.genAttrs systems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
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
          grillDisplaySim = pkgs.writeShellApplication {
            name = "grill-display-sim";
            runtimeInputs = [
              pkgs.esphome
              pkgs.git
              sdl2
              sdl2.dev
              pkgs.openssl
              pkgs.openssl.dev
              pkgs.stdenv.cc
            ];
            text = ''
              export CPATH="${pkgs.openssl.dev}/include''${CPATH:+:$CPATH}"
              export LIBRARY_PATH="${pkgs.openssl.out}/lib''${LIBRARY_PATH:+:$LIBRARY_PATH}"
              cd "$(git rev-parse --show-toplevel)"
              exec esphome run tests/grill-display-host.yaml "$@"
            '';
          };
          grillDisplaySimPress = pkgs.writeShellApplication {
            name = "grill-display-sim-press";
            runtimeInputs = [
              (pkgs.python3.withPackages (ps: [ ps.aioesphomeapi ]))
              pkgs.git
            ];
            text = ''
              python3 "$(git rev-parse --show-toplevel)/projects/grill-display/sim_press.py" "$@"
            '';
          };
          grillDisplaySimPanel = pkgs.writeShellApplication {
            name = "grill-display-sim-panel";
            runtimeInputs = [
              (pkgs.python3.withPackages (ps: [
                ps.aioesphomeapi
                ps.tkinter
              ]))
              pkgs.git
            ];
            text = ''
              python3 "$(git rev-parse --show-toplevel)/projects/grill-display/sim_panel.py" "$@"
            '';
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

      checks = forSystems (
        { pkgs, treefmt, ... }:
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
                  ];
                };
              }
              ''
                ruff check --no-cache "$src"
                touch "$out"
              '';

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
        {
          validateConfigs,
          grillDisplaySim,
          grillDisplaySimPress,
          grillDisplaySimPanel,
          ...
        }:
        {
          validate-configs = {
            type = "app";
            program = lib.getExe validateConfigs;
            meta.description = "Run `esphome config` on every device configuration";
          };
          grill-display-sim = {
            type = "app";
            program = lib.getExe grillDisplaySim;
            meta.description = "Build and run the grill display host simulation";
          };
          grill-display-sim-press = {
            type = "app";
            program = lib.getExe grillDisplaySimPress;
            meta.description = "Press simulated inputs on the running grill display simulation";
          };
          grill-display-sim-panel = {
            type = "app";
            program = lib.getExe grillDisplaySimPanel;
            meta.description = "Control panel for the running grill display simulation";
          };
        }
      );

      devShells = forSystems (
        {
          pkgs,
          sdl2,
          treefmt,
          validateConfigs,
          grillDisplaySim,
          grillDisplaySimPress,
          grillDisplaySimPanel,
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
              treefmt.config.build.wrapper
              validateConfigs
              grillDisplaySim
              grillDisplaySimPress
              grillDisplaySimPanel
            ];
          };
        }
      );
    };
}
