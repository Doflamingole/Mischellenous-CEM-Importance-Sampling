import numpy as np
from scipy.stats import beta, rv_continuous,kstest
from typing import List, Dict, Set, Tuple, Optional, Callable


def kl_divergence(p_dist, q_dist, num_points=1000):
    x = np.linspace(0.001, 0.999, num_points)  # avoid 0 and 1
    p = p_dist.pdf(x)
    q = q_dist.pdf(x)
    kl = np.sum(p * np.log(p / q)) * (x[1] - x[0])  # numerical integration
    return kl



def distribution_fitting_test(data, distribution: Optional[rv_continuous], expected_params: Dict, test_func: Optional[Callable]):
    if distribution is beta:
        x_a, x_b, loc, scale = beta.fit(data)
        a, b = expected_params["alpha"], expected_params["beta"]
        a_hat, b_hat, loc, scale = beta.fit(data)
        true_dist = beta(2,2)
        fitted_dist = beta(a_hat, b_hat)
        kl = kl_divergence(true_dist, fitted_dist)
        return kl

