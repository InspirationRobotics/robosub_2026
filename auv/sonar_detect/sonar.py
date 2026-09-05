"""Thin wrapper around the Blue Robotics Ping360, producing one sweep at a time.

Deliberately separate from auv/device/sonar/. What is borrowed from there is the
range/timing arithmetic in _configure(), because that is real physics that was
worked out correctly and there is nothing to gain from rederiving it. The class
structure is not borrowed.

One sweep in, one Sweep object out. Nothing runs in the background, nothing is
threaded, and the sonar object stays alive between calls - so repeated sweeps
cost only the sweep time, not a reconnection.
"""
import numpy as np

# brping is imported inside Sonar.__init__, not here, so that Sweep and anything
# that works on sweeps stays importable on a machine with no sonar library
# installed. That is what lets you analyse recorded sweeps on your laptop.

SPEED_OF_SOUND = 1500.0      # m/s. Fresh water is nearer 1480, salt nearer 1520;
                             # the 1-2% range error that causes is well below what
                             # we care about for a rough approach.
TICK = 25e-9                 # the sonar's sample-period clock tick, seconds
MAX_SAMPLES = 1000           # hardware cap on range bins per ping
MIN_SAMPLE_PERIOD = 400      # hardware floor on how fast it can sample
GRADIANS = 400               # a full circle, in the sonar's own angle units
DEG_PER_GRADIAN = 360.0 / GRADIANS


class Sweep:
    """One sweep's worth of data, plus everything needed to interpret it.

    image: 2D uint8 array. Rows are angle steps, columns are range bins,
           values are echo strength 0-255.
    """

    def __init__(self, image, angles_grad, metres_per_bin, forward_gradian):
        self.image = image
        self.angles_grad = angles_grad
        self.metres_per_bin = metres_per_bin
        self.forward_gradian = forward_gradian

    @property
    def step_grad(self):
        if len(self.angles_grad) < 2:
            return 1
        return abs(self.angles_grad[1] - self.angles_grad[0])

    def row_to_bearing_deg(self, row):
        """Image row -> bearing relative to the sub, degrees.
        Zero is straight ahead, positive is to starboard."""
        grad = self.angles_grad[int(round(row)) % len(self.angles_grad)]
        bearing = (grad - self.forward_gradian) * DEG_PER_GRADIAN
        return (bearing + 180.0) % 360.0 - 180.0     # wrap to (-180, 180]

    def col_to_range_m(self, col):
        """Image column -> distance in metres."""
        return col * self.metres_per_bin


class Sonar:
    def __init__(self, device=None, udp=None, baudrate=115200,
                 max_range_m=10.0, gain=2, transmit_freq=800,
                 forward_gradian=0):
        """
        device / udp: pick one. udp is a (host, port) pair and goes through
            pingproxy, which lets Ping Viewer stay connected at the same time -
            though whether pingproxy really serves two clients cleanly is worth
            testing before you rely on it.

        forward_gradian: THE calibration constant. Which of the sonar's own
            angle numbers points out the front of the sub. Everything this
            module reports as a bearing is measured from here, so if your whole
            picture looks rotated, this is the single number to change. The old
            driver's docstring implies the sweep is centred on 200, which hints
            the answer might be 200 rather than 0 - but that is a docstring, not
            a measurement. Check it with a target dead ahead.
        """
        from brping import Ping360

        self._ping = Ping360()
        if udp is not None:
            self._ping.connect_udp(*udp)
        elif device is not None:
            self._ping.connect_serial(device, baudrate)
        else:
            raise ValueError("give either device= for serial or udp=(host, port)")

        if not self._ping.initialize():
            raise RuntimeError("Ping360 did not initialize - is it powered? 11-25V, "
                               "separate from USB.")

        self.forward_gradian = forward_gradian
        self._configure(max_range_m, gain, transmit_freq)

    def _configure(self, max_range_m, gain, transmit_freq):
        """Work out sample count and timing for the range we want.

        The physics: to see max_range metres, sound travels there and back, so
        the sonar must listen for 2*max_range/speed_of_sound seconds. That
        listening time gets divided into range bins. number_of_samples and
        sample_period have to multiply out to exactly that listening time.
        """
        self.max_range_m = max_range_m

        n = int(min(MAX_SAMPLES,
                    2 * max_range_m / (TICK * MIN_SAMPLE_PERIOD * SPEED_OF_SOUND)))
        period = int(2 * max_range_m / (n * TICK * SPEED_OF_SOUND))
        duration = int(max(period * TICK / 400, (8000 * max_range_m) / SPEED_OF_SOUND))

        self._ping.set_transmit_frequency(transmit_freq)
        self._ping.set_gain_setting(gain)
        self._ping.set_number_of_samples(n)
        self._ping.set_sample_period(period)
        self._ping.set_transmit_duration(duration)

        self.n_samples = n
        self.metres_per_bin = max_range_m / n

    def sweep(self, start_grad=0, end_grad=399, step=1):
        """Sweep an arc and return a Sweep.

        Time cost scales with how many pings you ask for and how far you listen
        on each. A 50-gradian sector at step 2 is 25 pings; a full circle at
        step 1 is 400. Same hardware, 16x difference in how long you wait.
        """
        angles = list(range(int(start_grad), int(end_grad) + 1, int(step)))
        rows = []

        for grad in angles:
            response = self._ping.transmitAngle(grad % GRADIANS)
            data = getattr(response, "data", None)
            if data is None:
                rows.append(np.zeros(self.n_samples, dtype=np.uint8))
                continue
            row = np.frombuffer(data, dtype=np.uint8)
            if len(row) < self.n_samples:
                row = np.pad(row, (0, self.n_samples - len(row)))
            rows.append(row[: self.n_samples])

        image = np.vstack(rows) if rows else np.zeros((0, self.n_samples), dtype=np.uint8)
        return Sweep(image, angles, self.metres_per_bin, self.forward_gradian)
