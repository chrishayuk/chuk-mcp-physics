# Rapier Physics Service Makefile

.PHONY: help build run test docker-build docker-run fly-deploy fly-logs fly-status clean

help:
	@echo "Rapier Physics Service"
	@echo ""
	@echo "Available targets:"
	@echo "  build          - Build release binary"
	@echo "  run            - Run service locally"
	@echo "  test           - Run tests"
	@echo "  docker-build   - Build Docker image"
	@echo "  docker-run     - Run Docker container"
	@echo "  fly-deploy     - Deploy to Fly.io"
	@echo "  fly-logs       - View Fly.io logs"
	@echo "  fly-status     - Check Fly.io status"
	@echo "  clean          - Clean build artifacts"

build:
	cargo build --release

run:
	cargo run --release

test:
	cargo test

docker-build:
	docker build -t rapier-physics-service .

docker-run:
	docker run -p 9000:9000 rapier-physics-service

fly-deploy:
	fly deploy

fly-logs:
	fly logs

fly-status:
	fly status

clean:
	cargo clean
	rm -rf target/
