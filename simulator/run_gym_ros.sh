#!/bin/sh

container_name=f1tenth_gym_ros
create_container (){
        docker run --rm -it\
        --name ${container_name} \
        -h ${container_name} \
	--env DISPLAY=$DISPLAY\
        --privileged \
        --net=host \
        f1tenth:gym_ros_foxy \
        run_sim.sh
}

rm_container (){
	if [ "$(docker ps -aq -f name=${container_name})" ]
        then
		if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
		then
			docker stop ${container_name}
		fi
        	docker rm ${container_name}
        fi
}

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${container_name} bash 
else
	rm_container 
	xhost + 
	create_container 
	xhost -
fi