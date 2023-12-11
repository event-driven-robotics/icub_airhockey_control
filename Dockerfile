FROM eventdrivenrobotics/gazebo:latest

ARG SOURCE_FOLDER=/usr/local/src

RUN apt install -y libspdlog-dev libc6-dev nlohmann-json3-dev

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/robotology/idyntree.git --depth 1 && \
     cd idyntree && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` install

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/artivis/manif.git --depth 1 -b 0.0.4 && \
     cd manif && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` install

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/osqp/osqp --depth 1 && \
     cd osqp && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` install

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/robotology/osqp-eigen --depth 1 && \
     cd osqp-eigen && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` install

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/ami-iit/lie-group-controllers.git --depth 1 && \
     cd lie-group-controllers && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` install

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/ami-iit/bipedal-locomotion-framework.git --depth 1 && \
     cd bipedal-locomotion-framework && \
      # TODO make this change on github
     sed -i 's/#include <types.h>//g' src/IK/src/JointLimitsTask.cpp && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` install

RUN cd $SOURCE_FOLDER && \
     git clone https://github.com/event-driven-robotics/icub_airhockey_control.git --depth 1 && \
     cd icub_airhockey_control && \
     mkdir build && \
     cd build/ && \
     cmake .. && \
     make -j `nproc` 
     # TODO add target install