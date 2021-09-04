
function High = tank(V,S1,S2,A1,A2,h1,h2)

g = 980;       

High(1,:) =  (-A1*sqrt(2*g*h1)/S1)+V/(S1);
High(2,:) =  (A1*sqrt(2*g*h1)/S2)-(A2*sqrt(2*g*h2))/S2;
       
end