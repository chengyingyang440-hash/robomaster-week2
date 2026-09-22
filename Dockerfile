FROM qzhhhi/rmcs-develop@sha256:b63c90cb547234ed323d9bae016b4b64208c6f91361bf4782eea8d121f72b9af

USER root
RUN printf '\nsource /opt/ros/jazzy/setup.bash\n' >> /etc/bash.bashrc

USER ubuntu
WORKDIR /workspace
