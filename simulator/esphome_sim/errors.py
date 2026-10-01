"""Errors that the command line reports as a single line."""


class SimError(Exception):
    """A problem caused by user input or project files."""


class ConnectError(SimError):
    """The simulation is not reachable or the connection dropped."""
