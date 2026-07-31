#!/usr/bin/env python3
import rospy
import docker
import json
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from std_msgs.msg import String


def get_own_container_id():
    hostname = os.environ.get('HOSTNAME', '')
    if hostname:
        return hostname
    try:
        with open('/proc/self/cgroup') as f:
            for line in f:
                if 'docker' in line or 'containerd' in line:
                    return line.strip().split('/')[-1][:12]
    except Exception:
        pass
    return None


def compute_cpu_percent(stats):
    try:
        cpu_delta = (stats['cpu_stats']['cpu_usage']['total_usage']
                     - stats['precpu_stats']['cpu_usage']['total_usage'])
        system_delta = (stats['cpu_stats']['system_cpu_usage']
                         - stats['precpu_stats']['system_cpu_usage'])
        online_cpus = stats['cpu_stats'].get(
            'online_cpus',
            len(stats['cpu_stats']['cpu_usage'].get('percpu_usage', []) or [1])
        )
        if system_delta > 0 and cpu_delta > 0:
            return round((cpu_delta / system_delta) * online_cpus * 100.0, 2)
    except (KeyError, TypeError, ZeroDivisionError):
        pass
    return 0.0


def compute_memory(stats):
    try:
        usage = stats['memory_stats']['usage']
        cache = stats['memory_stats'].get('stats', {}).get('cache', 0)
        limit = stats['memory_stats']['limit']
        real_usage = usage - cache
        percent = round((real_usage / limit) * 100.0, 2) if limit else 0.0
        return round(real_usage / (1024 * 1024), 2), percent
    except (KeyError, TypeError, ZeroDivisionError):
        return 0.0, 0.0


def compute_network(stats):
    rx = tx = 0
    for iface in stats.get('networks', {}).values():
        rx += iface.get('rx_bytes', 0)
        tx += iface.get('tx_bytes', 0)
    return round(rx / (1024 * 1024), 3), round(tx / (1024 * 1024), 3)


def fetch_one(container):
    """Runs in a worker thread: blocking stats() call + metric extraction."""
    stats = container.stats(stream=False)
    mem_mb, mem_pct = compute_memory(stats)
    rx_mb, tx_mb = compute_network(stats)

    # Main process PID lives in the container's low-level attrs, not stats
    pid = container.attrs.get('State', {}).get('Pid', 0)

    return container.name, {
        "pid": pid,
        "cpu_percent": compute_cpu_percent(stats),
        "memory_mb": mem_mb,
        "memory_percent": mem_pct,
        "pids": stats.get('pids_stats', {}).get('current', 0),
        "net_rx_mb": rx_mb,
        "net_tx_mb": tx_mb,
    }


def detailed_monitor():
    pub = rospy.Publisher('node_metrics', String, queue_size=10)
    rospy.init_node('node_monitor', anonymous=True)
    rate = rospy.Rate(0.5)  # every 2 seconds

    client = docker.from_env()
    own_id = get_own_container_id()
    rospy.loginfo("node_monitor running as container id: %s", own_id)

    executor = ThreadPoolExecutor(max_workers=16)

    while not rospy.is_shutdown():
        report = {}

        try:
            containers = client.containers.list()
        except Exception as e:
            rospy.logwarn_throttle(10, "Docker API error: %s", e)
            rate.sleep()
            continue

        targets = [c for c in containers
                   if not (own_id and c.id.startswith(own_id))]

        futures = {executor.submit(fetch_one, c): c for c in targets}
        for future in as_completed(futures):
            container = futures[future]
            try:
                name, metrics = future.result()
                report[name] = metrics
            except Exception as e:
                rospy.logwarn_throttle(
                    10, "Failed to get stats for %s: %s", container.name, e)

        # Sort by PID ascending before publishing.
        # dict insertion order is preserved by json.dumps in Python 3.7+.
        report = dict(sorted(report.items(), key=lambda item: item[1]["pid"]))

        pub.publish(json.dumps(report))
        rate.sleep()


if __name__ == '__main__':
    try:
        detailed_monitor()
    except rospy.ROSInterruptException:
        pass