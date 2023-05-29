# This message expresses a weighted constraint with lower/upper tolerances

# Note: Never use +inf/-inf

float64 value # valid values are between [value+lower, value+upper]
float64 lower_tolerance # nan disables this boundary. set lower==upper to hard-bound. set lower<upper to soft-bound.
float64 upper_tolerance # nan disables this boundary. set lower==upper to hard-bound. set lower<upper to soft-bound.
float64 weight # 0 to disable this constraint completely. valid values are between [0, +inf). set to 1.0 if not decided.