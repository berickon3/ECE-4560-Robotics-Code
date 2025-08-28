function point = planarTranslation2d(point,displacement,theta)
    point = displacement + [cos(theta), -sin(theta); sin(theta), cos(theta)]*point;
end