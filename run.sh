xhost +local:docker  # (enables X11 forwarding to Docker; only needed once)
docker run --rm -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)/data:/app/data" \
  navstream data/2011_10_03/2011_10_03_drive_0042_unsync_gps_loss/oxts oxts_out.csv ekf_out.csv