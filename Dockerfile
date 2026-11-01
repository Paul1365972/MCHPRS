# Build stage
FROM lukemathwalker/cargo-chef:latest-rust-alpine AS chef

RUN apk add --no-cache openssl-dev openssl-libs-static

WORKDIR /app/

FROM chef AS planner
COPY ./src/ ./src/
COPY ./crates/ ./crates/
COPY ./Cargo.toml ./Cargo.lock ./
RUN cargo chef prepare --recipe-path recipe.json

FROM chef AS builder
COPY ./rust-toolchain.toml ./
COPY --from=planner /app/recipe.json ./
RUN cargo chef cook --locked --release --recipe-path recipe.json
COPY ./src/ ./src/
COPY ./crates/ ./crates/
COPY ./Cargo.toml ./Cargo.lock ./
RUN cargo build --locked --release

# Runtime stage
FROM scratch

COPY --from=builder /app/target/release/mchprs /

VOLUME ["/data/"]
WORKDIR /data/
EXPOSE 25565
ENTRYPOINT ["/mchprs"]
