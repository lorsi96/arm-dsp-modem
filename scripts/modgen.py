import numpy as np
import fxpmath
import matplotlib.pyplot as plt

def rrc_coeffs(sz:int, beta:float=0.5, n_lobes:int=5):
    N = sz * (n_lobes * 2 + 1)
    t = (np.arange(N) - N / 2) / sz

    h_rrc = np.zeros(len(t), dtype=float)
    sample_i = np.zeros(len(t), dtype=bool)
    subi = t == 0
    sample_i = np.bitwise_or(sample_i, subi)
    h_rrc[subi] = 1.0 - beta + (4 * beta / np.pi)
    subi = np.abs(t) == 1 / (4 * beta)
    sample_i = np.bitwise_or(sample_i, subi)
    h_rrc[subi] = (beta / np.sqrt(2)) \
                * (((1 + 2 / np.pi) * (np.sin(np.pi / (4 * beta))))
                + ((1 - 2 / np.pi) * (np.cos(np.pi / (4 * beta)))))
    sample_i = np.bitwise_not(sample_i)
    ti = t[sample_i]
    h_rrc[sample_i] = np.sin(np.pi * ti * (1 - beta)) \
                    + 4 * beta * ti * np.cos(np.pi * ti * (1 + beta))
    h_rrc[sample_i] /= (np.pi * ti * (1 - (4 * beta * ti) ** 2))
    return h_rrc

def float_arr_to_q15(arr):
    return [fxpmath.Fxp(v, signed=True, n_word=16, n_frac=14).hex() for v in arr]


def nparr_to_c_header(name, arr):
    start = '''#ifndef __H_RRC__\n#define __H_RRC__\n\n#include "arm_math.h\n\n"''' 
    arr = ',\n\t'.join(arr)
    arr_decl = f"q15_t {name}[] = \n\t""{" + arr + "}\n"
    return start + arr_decl + "\n#endif\n"

if __name__ == '__main__':
    coeffs = float_arr_to_q15(rrc_coeffs(16, n_lobes=3))
    header = nparr_to_c_header('rrc_coeffs', coeffs)
    # plt.plot(list(map(lambda s: s if s <= 0x7FFF else s - 0xFFFF, [int(a, 0) for a in coeffs])))
    # plt.show()
    print(header)