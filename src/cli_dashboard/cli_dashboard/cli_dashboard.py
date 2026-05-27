#!/usr/bin/env python3
"""
Member 5 — CLI Dashboard
Operator control panel using the click library.
Commands: status, set-phase, estop, record
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
        '/obstacles/pose',
        '/emergency_stop',
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
    subprocess.run([
        'ros2', 'topic', 'pub', '--once',
        '/emergency_stop', 'std_msgs/msg/Bool', '{data: true}'
    ], check=False)
    click.echo('Emergency stop published.')


def main():
    cli()


if __name__ == '__main__':
    main()
