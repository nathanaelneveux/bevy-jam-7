# Bevy Jam 7

Game jam project for **Bevy Jam 7**.

## Development

### Run locally

```sh
cargo run
```

### Quality checks

```sh
cargo test
cargo clippy -- -D warnings
cargo fmt --all -- --check
```

## Web / Wasm

This project is set up to support `wasm32-unknown-unknown`.

```sh
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
```

To generate web bindings manually:

```sh
version=$(cargo metadata --format-version 1 | jq --raw-output '.packages[] | select(.name=="wasm-bindgen") | .version')
cargo install -f wasm-bindgen-cli --version "$version"
wasm-bindgen --no-typescript --out-name bevy_game --out-dir wasm --target web target/wasm32-unknown-unknown/release/bevy_new_minimal.wasm
```

Then serve the `wasm/` directory with a local HTTP server.

## Dependency Notes

- `bevy_voxel_world` is pinned to `nathanaelneveux/bevy_voxel_world` branch `wasm-support`.
- `avian3d` is pinned to an Avian `main` revision that includes MoveAndSlide work while remaining compatible with the current Bevy 0.18 stack.

## CI and Release

- CI workflow: `.github/workflows/ci.yaml`
- Release workflow: `.github/workflows/release.yaml`

Set `env.binary` in `.github/workflows/release.yaml` to match your crate binary (currently `bevy_new_minimal`).
