function stability = checkLongitudinalStability(coeff, Kn)

stability = Kn >= -coeff.Cma/coeff.CLa;