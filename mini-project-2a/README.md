# Mini-Project 2A

## Classification of Accelerometer Data Using the micro:bit

---

## Project Overview

This project develops a complete workflow for classifying accelerometer data recorded with a BBC micro:bit.

The goal is to determine whether the device is:

- Held steady (Rest)
- Being shaken (Shake)

We record approximately 20 seconds of accelerometer data and train a machine learning model (logistic regression) to classify the movement state.

---

## Movement Protocol

The following sequence was recorded:

| Time (seconds) | Action      |
| -------------- | ----------- |
| 0–5            | Hold steady |
| 5–10           | Shake       |
| 10–15          | Hold steady |
| 15–20          | Shake       |

This produces labeled rest and shake intervals for training and testing.

---

## Data Collection

Accelerometer data was recorded using the micro:bit data logging functionality.

Data can be transferred via:

- USB serial connection
- MakeCode data logger
- Python script
- Radio transmission

The accelerometer outputs values in milligravities (mg).

The dataset is saved as:

shake_data.csv

---

## Data Processing Pipeline

### 1. Data Cleaning

- Select columns: time, x, y, z
- Rename to: t, ax, ay, az
- Drop missing values
- Convert mg → m/s²

Conversion factor:

1 mg = 0.00980665 m/s²

### 2. Outlier Handling

Extreme acceleration values are replaced with the median to reduce noise.

### 3. Compute Velocity and Position

Velocity and position are estimated using numerical integration:

- Velocity = integral of acceleration
- Position = integral of velocity

(using scipy.integrate.cumulative_trapezoid)

---

## Train/Test Split

Training uses the first rest and first shake:

Train Rest: 0–5 s  
Train Shake: 5–10 s

Testing uses the second rest and second shake:

Test Rest: 10–15 s  
Test Shake: 15–20 s

Labels:

- 0 = Rest
- 1 = Shake

---

## Models Used

Two logistic regression models were trained.

### Model 1 (Includes Time)

p̂ = σ(b₀ + bₜ t + bₓ ax + b_y ay + b_z az)

Features:

- t
- ax
- ay
- az

### Model 2 (Acceleration Only)

p̂ = σ(b₀ + bₓ ax + b_y ay + b_z az)

Features:

- ax
- ay
- az

---

## Results

| Model               | Train Accuracy | Test Accuracy |
| ------------------- | -------------- | ------------- |
| Model 1 (with time) | 1.00           | 0.49          |
| Model 2 (no time)   | 0.80           | 0.86          |

---

## Explanation of Results

Model 1 uses time as a feature.

Because rest and shake occur in fixed time intervals, the model learns a time-based decision boundary instead of learning the actual motion behavior. It memorizes when shaking happened rather than how the device moved.

This results in:

- Perfect training accuracy (overfitting)
- Poor test accuracy

Model 2 does not use time and instead relies only on acceleration values. It learns a physically meaningful boundary based on motion characteristics, resulting in better generalization.

---

## How to Run This Project

### 1. Install Dependencies

pip install numpy pandas matplotlib scipy scikit-learn

### 2. Place Dataset

Ensure shake_data.csv is in the same directory as the notebook.

### 3. Run the Notebook

Execute all cells in:

accelerometer_classification.ipynb

The notebook will:

- Preprocess the data
- Train both models
- Print accuracy
- Generate visualizations

---

## Suggestions for Improvement

To improve classification accuracy:

### Feature Engineering

- Use acceleration magnitude: sqrt(ax² + ay² + az²)
- Add rolling window statistics (variance, standard deviation)
- Extract frequency-domain features (FFT)

### Better Models

- Support Vector Machines
- Random Forest
- K-Nearest Neighbors
- Neural Networks

### More Data

- Record multiple sessions
- Randomize rest/shake order
- Use cross-validation

### Real-Time Deployment

- Implement threshold-based classification using acceleration magnitude
- Deploy a simplified model directly on the micro:bit

---

## Final Conclusion

This project demonstrates:

- A complete sensor-to-model workflow
- The importance of proper feature selection
- The risk of overfitting
- How physically meaningful features improve generalization

Model 2 (acceleration-only) provides the best balance of interpretability and generalization for this task.

---

## Project Structure

.
├── shake_data.csv  
├── accelerometer_classification.ipynb  
└── README.md
