#!/bin/bash
COMPOSE_FILE="docker-compose.yaml"
SERVICE_NAME="$1"
CONTAINER_NAME="forzaeth_devcontainer_${SERVICE_NAME}_$USER"

if [ -z "$SERVICE_NAME" ]; then
    echo "Error: Service name not provided."
    exit 1
fi

# Check if the container exists
CONTAINER_STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null)
EXIT_CODE=$?

# Stop any lingering noVNC process
docker ps --filter name=forzaeth_novnc_ --format '{{.Names}}' | grep -vx "forzaeth_novnc_$USER" | xargs -r docker stop

if [ $EXIT_CODE -eq 0 ]; then
    # Warn if the current container is stale
    CONTAINER_IMAGE=$(docker inspect -f '{{.Image}}' "$CONTAINER_NAME")
    CURRENT_IMAGE=$(docker image inspect -f '{{.Id}}' "race_stack:humble_${SERVICE_NAME}_$USER" 2>/dev/null)
    if [ -n "$CURRENT_IMAGE" ] && [ "$CONTAINER_IMAGE" != "$CURRENT_IMAGE" ] || \
       ! docker inspect -f '{{range .Mounts}}{{.Name}} {{end}}' "$CONTAINER_NAME" | grep -qw "race_stack_ws_$USER"; then
        echo -e "\033[33mWarning: container is stale (older image or missing workspace volume). Run 'make relaunch' to recreate it.\033[0m"
    fi
    if [ "$CONTAINER_STATUS" == "running" ]; then
        echo "Container is already running."
        docker compose -f "$COMPOSE_FILE" up -d novnc
        docker compose -f "$COMPOSE_FILE" exec "$SERVICE_NAME" /bin/bash
    elif [ "$CONTAINER_STATUS" == "exited" ]; then
        echo "Container is stopped. Restarting..."
        docker compose -f "$COMPOSE_FILE" start "$SERVICE_NAME"
        docker compose -f "$COMPOSE_FILE" up -d --no-recreate novnc
    fi
    echo "Done. To attach: docker compose exec $SERVICE_NAME /bin/bash"
else
    source ".devcontainer/xauth_setup.sh"
    DISPLAY=${DISPLAY} docker compose -f "$COMPOSE_FILE" up -d "$SERVICE_NAME" novnc
    echo "Waiting for container to initialize..."
    sleep 3
    docker exec -u "$USER" "$CONTAINER_NAME" /bin/bash -c "/home/$USER/ws/src/race_stack/.install_utils/post_create_command.sh"
    echo "To attach: docker compose exec $SERVICE_NAME /bin/bash or run make launch again"
fi
