FROM mxwilliam/mxck:mxck-melodic-base-l4t-35.2.1 

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

WORKDIR ./melodic_ws