FROM wpilib/roborio-cross-ubuntu:2025-24.04
RUN apt-get update -y && apt-get upgrade -y && apt-get install python3-venv openssh-client -y
RUN python3 -m venv /venv
RUN /venv/bin/pip install wpiformat
ENV PATH="/venv/bin:$PATH"