from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    'Duration',
]


class Duration(object):
    """Duration consists of two integers: seconds and nanoseconds.

    Attributes
    ----------
    secs: int
        Integer representing number of seconds.
    nsecs: int
        Integer representing number of nanoseconds.
    """

    def __init__(self, secs, nsecs):
        self.secs = int(secs)
        self.nsecs = int(nsecs)

    def __str__(self):
        return 'Duration({}, {})'.format(self.secs, self.nsecs)

    def __repr__(self):
        return self.__str__()

    @property
    def seconds(self):
        """Returns the duration as floating-point seconds.

        Returns
        -------
        float
            Floating-point seconds
        """
        return self.secs + 1e-9 * self.nsecs

    @classmethod
    def from_seconds(cls, seconds):
        """Construct a duration from a float second (e.g. 535.66726)

        Parameters
        ----------
        seconds : float
        """
        return cls(int(seconds), (seconds - int(seconds))*1e9)

    @classmethod
    def from_data(cls, data):
        """Construct a duration from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`Duration`
             An instance of :class:`Duration`.
        """
        duration = cls(0, 0)
        duration.data = data
        return duration

    def to_data(self):
        """Return the data dictionary that represents the duration, and from
        which it can be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the duration."""
        return {
            'secs': self.secs,
            'nsecs': self.nsecs
        }

    @data.setter
    def data(self, data):
        self.secs = data.get('secs') or 0
        self.nsecs = data.get('nsecs') or 0
