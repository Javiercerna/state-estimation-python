# Extended Kalman Filter for GPS and Odometry fusion

EKF fusing GPS and odometry measurements. The results are compared to ground truth measurements obtained by a RTK GPS.
The dataset used comes from the following source:

http://ipds.univ-bpclermont.fr/index.php

## Installation

1. Install python 3.
2. Install the requirements using pip:
```
pip install -r requirements.txt
```
##

## Usage examples

To take a look at the dataset:

```
python check_dataset.py
```

![Image: check dataset results](./check_dataset.png)


To run the EKF on the dataset:

```
python run_filter.py
```

![Image: run filter results](./run_filter_results.png)

![Image: run filter errors](./run_filter_errors.png)
