COMPOSE = docker compose
SERVICE = ros2-jazzy

# ── Container management ──────────────────────────────────────────────────────

build:
	$(COMPOSE) build

up:
	$(COMPOSE) up -d

shell:
	$(COMPOSE) exec $(SERVICE) bash

down:
	$(COMPOSE) down

rebuild:
	$(COMPOSE) down
	$(COMPOSE) build --no-cache
	$(COMPOSE) up -d

logs:
	$(COMPOSE) logs -f

# ── ROS 2 workspace ───────────────────────────────────────────────────────────

ros-build:
	$(COMPOSE) exec $(SERVICE) bash -c \
		"cd /ros2_ws && colcon build --symlink-install"

ros-build-pkg:
	$(COMPOSE) exec $(SERVICE) bash -c \
		"cd /ros2_ws && colcon build --symlink-install --packages-select $(PKG)"

ros-nodes:
	$(COMPOSE) exec $(SERVICE) bash -c \
		"source /opt/ros/jazzy/setup.bash && ros2 node list"

ros-topics:
	$(COMPOSE) exec $(SERVICE) bash -c \
		"source /opt/ros/jazzy/setup.bash && ros2 topic list"

# ── Visualization ─────────────────────────────────────────────────────────────

rviz:
	xhost +local:docker
	$(COMPOSE) exec $(SERVICE) bash -c \
		"source /opt/ros/jazzy/setup.bash && rviz2"

# ── Backup ────────────────────────────────────────────────────────────────────

backup:
	docker commit ros2-jazzy-tutorials ros2-jazzy-tutorials:$(shell date +%Y%m%d)
	@echo "Saved snapshot: ros2-jazzy-tutorials:$(shell date +%Y%m%d)"

snapshots:
	docker images ros2-jazzy-tutorials

.PHONY: build up shell down rebuild logs ros-build ros-build-pkg \
        ros-nodes ros-topics rviz backup snapshots