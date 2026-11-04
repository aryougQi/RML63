FROM rml63/noetic-sim:local
SHELL ["/bin/bash", "-c"]
WORKDIR /workspace
COPY src /workspace/src
COPY build.sh verify_package.py /workspace/
RUN bash build.sh
COPY simulation /workspace/simulation
ENTRYPOINT ["/bin/bash", "/workspace/simulation/env.sh"]
CMD ["roslaunch", "rml63_fixed_sim", "scene.launch"]
