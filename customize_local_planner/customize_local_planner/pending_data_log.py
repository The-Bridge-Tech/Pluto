"""
Struct/container class for pending data to log
Author: Matthew Lauriault
Created: 11/8/24
"""


class PendingDataLog:

    """Struct class for logging pending data."""

    def __init__(self):
        self.clear()

    def clear(self):
        """Reset data log."""
        self.pending_data = []

    def add(self, pending_data_name: str):
        """Add name of pending data to log."""
        self.pending_data.append(pending_data_name)

    # def update(self, pending_data: dict):
    #     """Update log with dictionary composed of `{data name: still_pending}`."""
    #     pass

    def has_pending_data(self) -> bool:
        """Return if there is still data that is pending."""
        return len(self) > 0

    def __len__(self) -> int:
        return len(self.pending_data)
    
    def __str__(self) -> str:
        return "Waiting for: " + ", ".join(self.pending_data)
    
    def __repr__(self) -> str:
        return self.__str__()