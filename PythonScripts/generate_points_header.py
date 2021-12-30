import json
import numpy as np
import sys


def make_header(points, header_path):
    """
    :param points: (N x 3) Numpy array containing points to turn into a header.
    """

    with open(header_path, 'w', newline='') as header_file:
        header_file.write("#pragma once\n")
        header_file.write("#include <Arduino.h>\n\n")

        header_file.write("namespace LEDPoints\n{\n")
        header_file.write("\tconst float pointsFloat[3*%d] PROGMEM = {\n" % len(points))

        # float point definitions
        for p in points:
            if np.isnan(p[0]):
                header_file.write("\t\tNAN, NAN, NAN,\n")
            else:
                header_file.write("\t\t%f, %f, %f,\n" % (p[0], p[1], p[2]))

        header_file.write('\t};\n\n')

        # float getter
        header_file.write("\tconst float* const getLEDFloat(int nLedIdx)\n")
        header_file.write("\t{\n")
        header_file.write("\t\treturn &(pointsFloat[nLedIdx * 3]);\n")
        header_file.write("\t}\n\n")

        # 16.16 fixed point coordinates
        # (Note: For perlin simplex noise, 1.0 is the size of the Perlin cubes)
        fixedOffset = 2**(16 + 4)
        fixedScale = 2**(16 + 2) * 0.5

        header_file.write("const uint32_t points16p16[3*%d] PROGMEM = {\n" % len(points))
        for p in points:
            if np.isnan(p[0]):
                header_file.write("\t\t0, 0, 0,\n")
            else:
                p_fixed = np.round(p * fixedScale + fixedOffset).astype(np.uint32)
                header_file.write("\t\t%d, %d, %d,\n" % (p_fixed[0], p_fixed[1], p_fixed[2]))

        header_file.write('\t};\n\n')

        # 16p16 getter
        header_file.write("\tconst uint32_t* const getLED16p16(int nLedIdx)\n")
        header_file.write("\t{\n")
        header_file.write("\t\treturn &(points16p16[nLedIdx * 3]);\n")
        header_file.write("\t}\n\n")

        header_file.write('}\n')


def main():
    json_path = sys.argv[1]
    header_path = sys.argv[2]

    with open(json_path) as json_file:
        json_blob = json.load(json_file)

        points = np.array(json_blob["leds"], dtype=np.float)

        make_header(points, header_path)


if __name__ == "__main__":
    main()
