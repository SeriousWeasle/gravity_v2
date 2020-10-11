class comp:
    #init complex number
    def __init__(self, real, imag):
        self.real = real
        self.imag = imag
    
    #add 2 complex numbers
    def __add__(self, other):
        #add complex with other complex
        if isinstance(other, comp):
            r = self.real + other.real
            i = self.imag + other.imag
            return comp(r, i)
        #add complex with imaginary
        if isinstance(other, imag):
            i = self.imag + other.val
            return comp(self.real, i)
        #add complex with real
        else:
            r = self.real + other
            return comp(r, self.imag)
    
    #subtract 2 complex numbers
    def __sub__(self, other):
        #subtract complex from other complex
        if isinstance(other, comp):
            r = self.real - other.real
            i = self.imag - other.imag
            return comp(r, i)
        #subtract imaginary from complex
        if isinstance(other, imag):
            i = self.imag - other.val
            return comp(self.real, i)
        #subtract real from complex
        else:
            r = self.real - other
            return comp(r, self.imag)
    
    #multiply for complex numbers
    def __mul__(self, other):
        #comp * comp
        if isinstance(other, comp):
            r = (self.real * other.real) - (self.imag * other.imag)
            i = (self.imag * other.real) + (self.real * other.imag)
            return comp(r, i)
        #comp * imag
        if isinstance(other, imag):
            r = -(self.imag * other.val)
            i = (self.real * other.val)
            return comp(r, i)
        #comp * real
        else:
            r = self.real * other
            i = self.imag * other
            return comp(r, i)
    
    #define print and to string method
    def __str__(self):
        return str(self.real) + " + " + str(self.imag) + "i"


class imag:
    #initialize imaginary number
    def __init__(self, value):
        self.val = value
    
    #add
    def __add__(self, other):
        #imag + imag
        if isinstance(other, imag):
            #return imag
            return imag(self.val + other.val)
        #imag + comp
        if isinstance(other, comp):
            #return complex
            return comp(other.real, other.imag + self.val)
        #imag + real
        else:
            #return complex
            return comp(other, self.val)
    
    #subtract
    def __sub__(self, other):
        #imag - imag
        if isinstance(other, imag):
            return imag(self.val - other.val)
        #imag - complex
        if isinstance(other, comp):
            return comp(-other.real, self.val - other.imag)
        #imag - real
        else:
            return comp(-other, self.val)
    
    #multiply
    def __mul__(self, other):
        #imag * imag
        if isinstance(other, imag):
            return -1 * self.val * other.val
        #imag * complex
        if isinstance(other, comp):
            r = -1 * self.val * other.imag
            i = self.val * other.real
            return comp(r, i)
        #imag * real
        else:
            return imag(self.val * other)
    
    #define print and tostring method
    def __str__(self):
        return str(self.val) + "i"