import math



def main():
    fc = 1e3
    print('fc=', fc)
    print()
    a1, a2, b1, b2 = 2.5904, 0.3039, 4.1301, 1.1697

    print('a1=', a1)
    print('b1=', b1)

    C11 = 1 * 10 ** (-9)
    print('C11=', C11)
    C21 = C11 * 4 * b1 / (a1 ** 2) + 10e-9
    print('C21=', C21)

    R11 = (a1 * C21 - math.sqrt(math.pow(a1, 2) * math.pow(C21, 2) - 4 * b1 * C11 * C21)) / (
                4 * math.pi * fc * C11 * C21)
    print('R11=', R11)
    R21 = (a1 * C21 + math.sqrt(math.pow(a1, 2) * math.pow(C21, 2) - 4 * b1 * C11 * C21)) / (
                4 * math.pi * fc * C11 * C21)
    print('R21=', R21)

    print()

    print('a2=', a2)
    print('b2=', b2)
    C12 = 820 * 10 ** (-12)
    print('C12=', C12)
    C22 = C12 * 4 * b2 / (a2 ** 2) + 10e-9
    print('C22=', C22)

    R12 = (a2 * C22 - math.sqrt(math.pow(a2, 2) * math.pow(C22, 2) - 4 * b2 * C12 * C22)) / (
                4 * math.pi * fc * C12 * C22)
    print('R12=', R12)
    R22 = (a2 * C22 + math.sqrt(math.pow(a2, 2) * math.pow(C22, 2) - 4 * b2 * C12 * C22)) / (
                4 * math.pi * fc * C12 * C22)
    print('R22=', R22)


if __name__ == '__main__':
    main()
