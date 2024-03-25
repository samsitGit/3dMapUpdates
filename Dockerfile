#Config for a cmake project containing pcl library

FROM ubuntu:18.04

# Install dependencies

RUN set -ex; \
    apt-get update; \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    make \

RUN mkdir /app

WORKDIR /app

COPY . .

RUN mkdir build

WORKDIR /app/build

RUN cmake ..

RUN make

CMD ["./3dMapUpdates"]


