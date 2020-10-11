import math

def scaleSqrt(val, min_in, max_in, min_out, max_out):
    #make input into float between 0 and 1
    frac = math.sqrt(val - min_in)/math.sqrt(max_in-min_in)
    #multiply by new range and add lower bound
    return (frac *(max_out - min_out)) + min_out

def scaleLinear(val, min_in, max_in, min_out, max_out):
    #make input into float between 0 and 1
    frac = (val - min_in)/(max_in - min_in)
    #scale to output
    return (frac * (max_out - min_out)) + min_out

def scaleQuad(val, min_in, max_in, min_out, max_out):
    #input -> float [0, 1]
    frac = ((val - min_in) * (val - min_in))/((max_in - min_in) * (max_in - min_in))
    #scale to output
    return (frac * (max_out - min_out)) + min_out

def scaleExp(val, exp, min_in, max_in, min_out, max_out):
    #input to float between 0 and 1
    frac = ((val - min_in) ** exp) / ((max_in - min_in) ** exp)
    return (frac * (max_out - min_out)) + min_out