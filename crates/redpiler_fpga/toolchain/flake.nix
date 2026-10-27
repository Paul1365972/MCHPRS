{
  description = "openXC7 toolchain of the fpga backend";

  inputs.openxc7.url = "github:openXC7/toolchain-nix/22327ebfb31d5fc93fd3b366c05657cb4d0d024d";

  outputs = { openxc7, ... }:
    let
      system = "x86_64-linux";
      pkgs = import openxc7.inputs.nixpkgs { inherit system; };
      tools = openxc7.packages.${system};
      python = pkgs.python312.withPackages (packages: with packages; [
        pyyaml textx simplejson intervaltree cython
      ]);
      # nextpnr routes on a chip database of its own, which it installs under this name.
      chipDatabase = device: pkgs.runCommand "nextpnr-chipdb-${device}" {
        nativeBuildInputs = [ python tools.nextpnr ];
      } ''
        mkdir -p $out/share/nextpnr/himbaechel/xilinx
        python3 ${tools.nextpnr}/share/nextpnr/himbaechel/uarch/xilinx/gen/xilinx_gen.py \
          --xray ${tools.prjxray-db}/artix7 --device ${device} --bba chipdb.bba
        bbasm -l chipdb.bba $out/share/nextpnr/himbaechel/xilinx/chipdb-${device}.bin
      '';
      # One prefix, so that the backend finds the data of a tool under `share` beside its `bin`.
      toolchain = pkgs.symlinkJoin {
        name = "openxc7-toolchain";
        paths = [
          tools.nextpnr tools.prjxray pkgs.yosys pkgs.openfpgaloader
          (chipDatabase "xc7a50t")
          (pkgs.linkFarm "prjxray-database" { "share/prjxray/database" = tools.prjxray-db; })
        ];
        nativeBuildInputs = [ pkgs.makeWrapper ];
        postBuild = ''
          wrapProgram $out/bin/fasm2frames --set PYTHONPATH ${pkgs.lib.concatStringsSep ":" [
            "${tools.prjxray}/usr/share/python3"
            "${tools.fasm}/${python.sitePackages}"
            "${python}/${python.sitePackages}"
          ]}
        '';
      };
    in {
      devShells.${system}.default = pkgs.mkShell {
        # Verilator builds a simulation with make, a C++ compiler and python3.
        packages = [ toolchain pkgs.verilator python pkgs.cargo pkgs.rustc ];
      };
    };
}
