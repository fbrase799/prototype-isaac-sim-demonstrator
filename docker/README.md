# RunPod image

Build the Isaac Sim container on GitHub Actions (`linux/amd64`) and pull it
on RunPod. Do not build this image on a Mac.

```text
GitHub repo
    → GitHub Actions (linux/amd64)
    → ghcr.io/<owner>/isaac-sim-demonstrator:latest
    → docker.io/<dockerhub-user>/prototype-isaac-sim-demonstrator:latest
    → RunPod RTX GPU
```

Files:

| Path | Role |
| --- | --- |
| `docker/Dockerfile.runpod` | NGC Isaac Sim 6.0.1 + SSH tools |
| `docker/entrypoint.runpod.sh` | SSH (`PUBLIC_KEY`) and keep the pod running |
| `.github/workflows/build-runpod-image.yml` | Build and push to GHCR and Docker Hub |

Prefer Docker Hub on RunPod if GHCR pulls are rate-limited.

## Get an NGC API key

GitHub Actions uses this key to `docker pull nvcr.io/nvidia/isaac-sim:6.0.1`.
It is not your NVIDIA email or password.

1. Create or sign in to an NVIDIA account at
   [ngc.nvidia.com/signin](https://ngc.nvidia.com/signin).
2. Open [NGC API Keys](https://ngc.nvidia.com/setup/api-key) (or click your
   avatar → **Setup** → **API Keys**).
3. Click **Generate Personal Key** (or **Generate API Key**).
4. Enable at least **NGC Catalog** (needed to pull public catalog images such
   as Isaac Sim). Copy the key once; NGC will not show it again.
5. If prompted, accept the Isaac Sim / Omniverse license on the
   [Isaac Sim container page](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
   for your NGC org.

That string is `NGC_API_KEY`. The Docker username is always the literal text
`$oauthtoken`, not your NVIDIA username.

Store it in GitHub: repo **Settings → Secrets and variables → Actions →
New repository secret**, name `NGC_API_KEY`, paste the key.

## Docker Hub token

The workflow also pushes
`<dockerhub-user>/prototype-isaac-sim-demonstrator`.

1. Sign in at [hub.docker.com](https://hub.docker.com).
2. Open **Account settings → Personal access tokens** (or
   [hub.docker.com/settings/security](https://hub.docker.com/settings/security)).
3. Create a token with **Read & Write**.
4. Add GitHub secrets:
   - `DOCKERHUB_USERNAME` — your Docker Hub username
   - `DOCKERHUB_TOKEN` — the access token (not your account password)

Create the empty repository `prototype-isaac-sim-demonstrator` on Docker Hub
first if your account does not allow auto-create on push.

## GitHub setup

1. Push this repository to GitHub.
2. Add secrets `NGC_API_KEY`, `DOCKERHUB_USERNAME`, and `DOCKERHUB_TOKEN`.
3. A push to `main` rebuilds the image only if `Dockerfile.runpod`,
   `entrypoint.runpod.sh`, or this workflow changed. Experiment scripts
   do not trigger a rebuild. Use **Actions → Build RunPod image** to
   run it by hand.
4. For GHCR: in GitHub **Packages**, make the package public, or create a
   PAT with `read:packages`. Docker Hub images can be pulled with the usual
   `docker pull` if the repo is public.

The NGC base image is large. If the hosted runner runs out of disk, re-run
the workflow or use a Linux self-hosted runner.

Setting `ACCEPT_EULA=Y` in the Dockerfile accepts the
[NVIDIA Omniverse license](https://docs.omniverse.nvidia.com/platform/latest/common/nvidia-omniverse-license-agreement.html).

## RunPod template

- GPU: RTX 4090 (or another RTX Ampere/Ada GPU)
- Container image (prefer Docker Hub):
  `docker.io/<dockerhub-user>/prototype-isaac-sim-demonstrator:latest`
- Fallback: `ghcr.io/<owner>/isaac-sim-demonstrator:latest`
- Expose SSH (port 22)
- `ACCEPT_EULA=Y` is already set in the image

SSH in, then clone this repo into `/workspace` and run from there (see the
project README). The image is Isaac Sim + SSH only; it does not bake in
`src/`. After a script change, `git pull` or `rsync` — you do not need a
new image.

## Local Linux build (optional)

Only on a Linux/amd64 host with Docker and an NGC login:

```bash
docker login nvcr.io
docker build --platform linux/amd64 -f docker/Dockerfile.runpod \
  -t isaac-sim-demonstrator:latest .
```

## Docker Compose on a GPU VM

If the machine has Docker + NVIDIA Container Toolkit and you want the stock
NGC image instead of the GHCR or Docker Hub image:

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
