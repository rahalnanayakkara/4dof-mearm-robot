import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("q_space.csv")
left = df['left']
right = df['right']

plt.plot(left, right)
plt.title("Workspace of robot")
plt.xlabel("left servo angle")
plt.ylabel("right servo angle")
plt.show()

print(df)