import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

def generate_hadamard_code(n):
    H = scipy.linalg.hadamard(n)
    Hs = np.vstack([H, -H])

    return -Hs


def main():
    # using "augmented hadamard code" notation

    # Block notation (n, k, d)_2
    n = 8  # block length
    k = int(np.log2(n))
    d = 2**(k-1)

    print(f"({2**k}, {k+1}, {2**(k-1)})_2")
    Hs = generate_hadamard_code(n)
    print(Hs)

    # Precomputed nibble sums
    HsSum = np.cumsum(Hs, axis=1)
    print("Precomputed Nibble Sum")
    print(HsSum)

    # 4x sequence for 16 bit message
    Hs4 = np.hstack([Hs, Hs, Hs, Hs])

    # Precomputed offset sums

    plt.imshow(Hs, cmap='gray')
    plt.xlabel("Codeword Bit")
    plt.ylabel("Message Value")

    # to RGB cycles
    rgb = np.zeros((Hs4.shape[0], Hs4.shape[1], 3))

    # the color cycle
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    for led_idx in range(Hs4.shape[0]):
        # initial color from LED index
        c = led_idx % 3

        for i in range(Hs4.shape[1]):
            c = ( c + Hs4[led_idx, i]) % 3

            rgb[led_idx, i] = colors[c]

    plt.figure()
    plt.imshow(rgb)
    plt.show()




if __name__ == "__main__":
    main()
