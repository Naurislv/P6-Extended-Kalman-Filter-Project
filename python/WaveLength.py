"""Calculate Wave lenght using speed of light."""

c = 299792458


def wl(f):
    """Calculate wave lenght im mm.

    Input: Frequency in Hertzs
    Return: Lambda wavelength in mm.
    """

    return c / f * 1000
