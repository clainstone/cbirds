{
  description = "A flock of birds in your terminal";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in
    {
      packages = forAllSystems (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
        in
        {
          default = self.packages.${system}.cbirds;

          cbirds = pkgs.stdenv.mkDerivation {
            pname = "cbirds";
            version = "1.6.0";
            src = pkgs.lib.cleanSource self;

            nativeBuildInputs = [ pkgs.installShellFiles ];
            makeFlags = [ "PREFIX=$(out)" ];

            doCheck = true;
            checkTarget = "test";

            postInstall = ''
              installShellCompletion --cmd cbirds \
                --bash <($out/bin/cbirds --completion bash) \
                --zsh <($out/bin/cbirds --completion zsh) \
                --fish <($out/bin/cbirds --completion fish)
            '';

            meta = {
              description = "A flock of birds in your terminal";
              homepage = "https://github.com/clainstone/cbirds";
              license = pkgs.lib.licenses.mit;
              mainProgram = "cbirds";
              platforms = systems;
            };
          };
        });
    };
}
