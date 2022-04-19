function result = PolyValue(coefficient,t,order)
    coef_num = size(coefficient,1);
    poly_order = coef_num - 1;
    result = 0;
    for i = 1:coef_num
        if(i <= coef_num - order)
            result = result + factorial(poly_order + 1 - i) / factorial(poly_order + 1 - i - order)...
                * coefficient(i) * t^(poly_order + 1 - i - order);
        end
    end
end