function wrapped = wrapToPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end