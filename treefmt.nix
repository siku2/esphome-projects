{ pkgs, lib, ... }:
{
  projectRootFile = "flake.nix";

  programs = {
    nixfmt.enable = true;

    clang-format.enable = true;

    ruff-format.enable = true;
    ruff-check.enable = true;
    ruff-check.priority = 1;
    ruff-format.priority = 2;

    prettier.enable = true;
    prettier.includes = lib.mkForce [
      "*.json"
      "*.md"
    ];

    # Prettier indents block sequences and cannot be told not to.
    dprint.enable = true;
    dprint.includes = lib.mkForce [ "*.yaml" ];
    dprint.settings = {
      plugins = pkgs.dprint-plugins.getPluginList (plugins: [ plugins.g-plane-pretty_yaml ]);
      yaml.indentBlockSequenceInMap = false;
    };
  };

  settings = {
    on-unmatched = "debug";

    formatter.ruff-check.options = [ "--no-cache" ];
    formatter.ruff-format.options = [ "--no-cache" ];
  };
}
