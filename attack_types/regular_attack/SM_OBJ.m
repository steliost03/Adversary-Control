function f = SM_OBJ(unew,B,ucurrent)

    f = norm(B*(unew-ucurrent));

end