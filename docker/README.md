# RunPod image

Build the Isaac Sim container on GitHub Actions (`linux/amd64`) and pull it
on RunPod. Do not build this image on a Mac.

```text
GitHub repo
    → GitHub Actions (linux/amd64)
    → ghcr.io/<owner>/isaac-sim-demonstrator:latest
    → RunPod RTX GPU
```

Files:

| Path | Role |
| --- | --- |
| `docker/Dockerfile.runpod` | NGC Isaac Sim 6.0.1 + SSH tools + `src/` |
| `docker/entrypoint.runpod.sh` | SSH (`PUBLIC_KEY`) and keep the pod running |
| `.github/workflows/build-runpod-image.yml` | Build and push to GHCR |

The image tag is lowercase `ghcr.io/<github-owner>/<repo>:latest`.

## GitHub setup

1. Push this repository to GitHub.
2. Add repository secret `NGC_API_KEY` (NGC API key; Docker username is
   `$oauthtoken`).
3. Push to `main`, or run **Actions → Build RunPod image**.
4. In GitHub **Packages**, make the package public, or create a PAT with
   `read:packages` for RunPod.

The NGC base image is large. If the hosted runner runs out of disk, re-run
the workflow or use a Linux self-hosted runner.

Setting `ACCEPT_EULA=Y` in the Dockerfile accepts the
[NVIDIA Omniverse license](https://docs.omniverse.nvidia.com/platform/latest/common/nvidia-omniverse-license-agreement.html).

## RunPod template

- GPU: RTX 4090 (or another RTX Ampere/Ada GPU)
- Container image: `ghcr.io/<owner>/isaac-sim-demonstrator:latest`
- Expose SSH (port 22)
- `ACCEPT_EULA=Y` is already set in the image

SSH in, then run the experiments from `/workspace/isaac-sim-demonstrator`
(see the project README). The image already contains `src/`. You can still
`git pull` or `rsync` over it for faster iteration.

## Local Linux build (optional)

Only on a Linux/amd64 host with Docker and an NGC login:

```bash
docker login nvcr.io
docker build --platform linux/amd64 -f docker/Dockerfile.runpod \
  -t isaac-sim-demonstrator:latest .
```

## Docker Compose on a GPU VM

If the machine has Docker + NVIDIA Container Toolkit and you want the stock
NGC image instead of the GHCR image:

```bash
docker login nvcr.io
docker pull nvcr.io/nvidia/isaac-sim:6.0.1
./docker/run.sh exp01
```

Confirm GPU passthrough:

```bash
nvidia-smi
docker run --rm --gpus all ubuntu nvidia-smi
```

`PRIVACY_CONSENT` defaults to `Y` in Compose; export `PRIVACY_CONSENT=N` to
opt out.
