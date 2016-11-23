"""Median Filter utilities for calculating bearing."""


class MedianFilter(object):
    """A filter for calculating the median of n latest numbers"""

    def __init__(self, size):
        self.size = size
        self.array = []

    def add(self, item):
        """Removes first-added item and appends new one."""
        if len(self.array) > self.size:
            self.array.pop(0)
        self.array.append(item)

    def median(self):
        """Computes and returns median element in array."""
        sorted_array = sorted(self.array)
        index = (len(self.array) - 1) // 2

        if len(self.array) % 2:
            return sorted_array[index]
        else:
            return (sorted_array[index] + sorted_array[index + 1]) / 2.0
