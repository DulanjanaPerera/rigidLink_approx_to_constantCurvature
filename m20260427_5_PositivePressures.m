function p_pos_norm = m20260427_5_PositivePressures(p)
% computes the positive normalized pressure

minP = min(p);
maxP = max(p);

p_pos = p-minP;
p_pos_norm = p_pos/max(p_pos) * maxP;

end