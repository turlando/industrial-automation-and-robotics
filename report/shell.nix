#!/usr/bin/env nix-shell

{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = [
    pkgs.gnumake

    ( pkgs.texlive.combine {
      inherit (pkgs.texlive)
        scheme-small
        latexmk
        biblatex
        biber
        lualatex-math
        libertinus-fonts libertinus-otf
      ;
    } )
  ];
}
