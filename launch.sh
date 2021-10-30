tag='motion_planning'
docker build . --tag $tag
docker run --rm -v $(pwd):/home -p 6080:6080 $tag
