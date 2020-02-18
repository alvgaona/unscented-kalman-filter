# Rubric Points

This document describes the rubric points met and a brief description on each one.

## Compiling and Testing

### Criteria

The submission must compile.

### Meets Specification

The project code must compile without errors using `cmake` and `make`.

## Code Efficiency

### Criteria

The methods in the code should avoid unnecessary calculations.

### Meets Specification

Your code does not need to sacrifice comprehension, stability, or robustness for speed.
However, you should maintain good and efficient coding practices when writing your functions.

Here are some things to avoid.
This is not a complete list, but there are a few examples of inefficiencies.

- **Running the exact same calculation repeatedly** when you can run it once, store the value and then reuse the value later.
- Loops that run **too many times**.
- **Creating unnecessarily complex data structures** when simpler structures work equivalently.
- **Unnecessary** control flow checks.

## Accuracy

### Criteria

`[px, py, vx, vy]` output coordinates must have an `RMSE <= [0.30, 0.16, 0.95, 0.70]` after running for **longer than 1 second**.

### Meets Specification

The simulation collects the position and velocity values that your algorithm outputs and they are compare to the ground truth data.
Your `[px, py, vx, vy]` RMSE should be less than or equal to the values `[0.30, 0.16, 0.95, 0.70]` after the simulator has ran for longer than 1 second.
The simulator will also display if RMSE values surpass the threshold.

## Follows the Correct Algorithm

### Criteria

Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

### Meets Specification

While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter.
As such, your project should follow the algorithm as described in the preceding lesson.
