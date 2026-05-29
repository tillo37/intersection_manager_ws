#!/usr/bin/env python3
"""
Member 5 — CLI Dashboard
Operator control panel using the click library.
Commands: status, set-phase, estop, record, pedestrian-status
"""

import subprocess
import sys
import click


@click.group()
def cli():
    """Intersection Manager operator CLI."""
    pass


@cli.command()
def status():
    """Show current node list and active topics."""
    click.echo(click.style('=== Node list ===', bold=True, fg='cyan'))
    subprocess.run(['ros2', 'node', 'list'], check=False)
    click.echo()
    click.echo(click.style('=== Topic list ===', bold=True, fg='cyan'))
    subprocess.run(['ros2', 'topic', 'list'], check=False)
    click.echo()
    click.echo(click.style('=== Traffic phase ===', bold=True, fg='cyan'))
    subprocess.run(
        ['ros2', 'topic', 'echo', '--once', '/traffic/phase'],
        check=False, timeout=3)


@cli.command(name='set-phase')
@click.option('--phase', required=True,
              type=click.Choice(['GREEN', 'RED', 'YELLOW'], case_sensitive=False),
              help='Target traffic phase')
def set_phase(phase):
    """Force traffic light to a specific phase via /set_phase service."""
    if phase.upper() != 'GREEN':
        click.echo(click.style(
            'Warning: /set_phase service only supports forcing GREEN. '
            'Sending request anyway.', fg='yellow'))
    click.echo(f'Calling /set_phase with data=True (→ GREEN)...')
    subprocess.run([
        'ros2', 'service', 'call', '/set_phase',
        'std_srvs/srv/SetBool', '{data: true}'
    ], check=False)


# ROS Function 1: record
@cli.command()
@click.option('--duration', default=30, show_default=True,
              help='Recording duration in seconds')
@click.option('--output', default='bags/scenario', show_default=True,
              help='Output bag directory')
def record(duration, output):
    """Record all key topics to a rosbag for the given duration."""
    topics = [
        '/vehicle/pose',
        '/vehicle/velocity',
        '/traffic/phase',
        '/obstacles/pose',     # Member 3 — pedestrian positions
        '/emergency_stop',     # Member 3 — danger zone state
        '/viz/markers',
    ]
    click.echo(click.style(
        f'Recording {len(topics)} topics for {duration} s → {output}', fg='green'))
    cmd = ['ros2', 'bag', 'record', '-o', output] + topics
    try:
        subprocess.run(cmd, timeout=duration + 2, check=False)
    except subprocess.TimeoutExpired:
        click.echo('Recording complete.')


# ROS Function 2: estop
@cli.command()
def estop():
    """Publish emergency stop (True) to /emergency_stop topic."""
    click.echo(click.style('!! PUBLISHING EMERGENCY STOP !!', fg='red', bold=True))
    # Publishes directly to Member 3's /emergency_stop topic —
    # allows operator to trigger a halt without waiting for a pedestrian event
    subprocess.run([
        'ros2', 'topic', 'pub', '--once',
        '/emergency_stop', 'std_msgs/msg/Bool', '{data: true}'
    ], check=False)
    click.echo('Emergency stop published.')


# Member 3 contribution: pedestrian-status
@cli.command(name='pedestrian-status')
def pedestrian_status():
    """Member 3 — Show live pedestrian positions and emergency stop state.

    Reads /obstacles/pose to display current x,y coordinates of all 6
    simulated pedestrians. Reads /emergency_stop to show whether any
    pedestrian is currently inside the 1.0 m danger zone.
    """

    # ── Pedestrian positions ──────────────────────────────────────────────────
    # /obstacles/pose is a PoseArray — one Pose per pedestrian, in the map frame
    # Published every 0.5 s by pedestrian_sim_node (Member 3)
    click.echo(click.style('=== Pedestrian Positions (Member 3) ===',
                           bold=True, fg='cyan'))
    subprocess.run(
        ['ros2', 'topic', 'echo', '--once', '/obstacles/pose'],
        check=False,
        timeout=3)   # 3 s timeout — if no message arrives the node is not running
    click.echo()

    # ── Emergency stop state ─────────────────────────────────────────────────
    # /emergency_stop is a Bool published every 0.5 s (heartbeat safety pattern)
    # True  → pedestrian inside DANGER_RADIUS (1.0 m) → all vehicles must halt
    # False → zone clear → intersection may resume normal operation
    click.echo(click.style('=== Emergency Stop State ===', bold=True, fg='cyan'))

    result = subprocess.run(
        ['ros2', 'topic', 'echo', '--once', '/emergency_stop'],
        capture_output=True, text=True, timeout=3)

    output = result.stdout.strip()

    if 'data: true' in output.lower():
        # Active emergency — red warning so operator notices immediately
        click.echo(click.style(
            '  !! EMERGENCY STOP ACTIVE — pedestrian in danger zone !!',
            fg='red', bold=True))
    elif 'data: false' in output.lower():
        # Zone clear — green confirmation
        click.echo(click.style(
            '  Zone clear — no pedestrians in danger zone.',
            fg='green'))
    else:
        # No message received — node is probably not running
        click.echo(click.style(
            '  (no message received — is pedestrian_sim_node running?)',
            fg='yellow'))

    click.echo(output)


def main():
    cli()


if __name__ == '__main__':
    main()
