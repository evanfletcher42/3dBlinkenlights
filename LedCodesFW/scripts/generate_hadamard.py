import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

def generate_hadamard_code(n):
    H = scipy.linalg.hadamard(n, dtype=np.uint8)
    Hs = np.vstack([H, -H])
    Hs = (1-Hs)//2

    plt.imshow(Hs, cmap='gray')
    plt.show()


def main():
    generate_hadamard_code(4)


if __name__ == "__main__":
    main()
