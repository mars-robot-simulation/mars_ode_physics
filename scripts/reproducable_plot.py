import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

import numpy as np
import pandas as pd

data = pd.read_csv('reproducable_test_log.csv')

data.plot(0, [1, 2, 3], subplots=True)
plt.show()
