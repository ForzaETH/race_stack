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

if [ $EXIT_CODE -eq 0 ]; then
    if [ "$CONTAINER_STATUS" == "running" ]; then
        echo "Container is already running."
        docker compose -f "$COMPOSE_FILE" exec "$SERVICE_NAME" /bin/bash
    elif [ "$CONTAINER_STATUS" == "exited" ]; then
        echo "Container is stopped. Restarting..."
        docker compose -f "$COMPOSE_FILE" start "$SERVICE_NAME"
    fi
    echo "Done. To attach: docker compose exec $SERVICE_NAME /bin/bash"
else
    source ".devcontainer/xauth_setup.sh"
    DISPLAY=${DISPLAY} docker compose -f "$COMPOSE_FILE" up -d "$SERVICE_NAME"
    echo "Waiting for container to initialize..."
    sleep 3
    docker exec -u "$USER" "$CONTAINER_NAME" /bin/bash -c "/home/$USER/ws/src/race_stack/.install_utils/post_create_command.sh"
    echo "To attach: docker compose exec $SERVICE_NAME /bin/bash or run make launch again"
fi