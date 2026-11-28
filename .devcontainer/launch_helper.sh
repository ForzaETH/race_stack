#!/bin/bash
SERVICE_NAME="race_stack"
CONTAINER_NAME="forzaeth_devcontainer_$USER"

bash .install_utils/initialize_command.sh
compose() { docker compose -f docker-compose.yaml -f .devcontainer/docker-compose.local.yaml "$@"; }

# Check if the container exists
CONTAINER_STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null)
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    if [ "$CONTAINER_STATUS" == "running" ]; then
        echo "Container is already running."
        compose up -d novnc
        compose exec "$SERVICE_NAME" /bin/bash
    elif [ "$CONTAINER_STATUS" == "exited" ]; then
        echo "Container is stopped. Restarting..."
        compose start "$SERVICE_NAME"
        compose up -d --no-recreate novnc
    fi
    echo "Done. To attach: make launch or docker compose exec $SERVICE_NAME /bin/bash"
else
    DISPLAY=${DISPLAY} compose up -d "$SERVICE_NAME" novnc
    echo "Waiting for container to initialize..."
    sleep 3
    docker exec -u "$USER" "$CONTAINER_NAME" /bin/bash -c "/home/$USER/ws/src/race_stack/.install_utils/post_create_command.sh"
    echo "To attach: docker compose exec $SERVICE_NAME /bin/bash or run make launch again"
fi
