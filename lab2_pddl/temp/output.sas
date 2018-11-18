begin_version
3
end_version
begin_metric
1
end_metric
12
begin_variable
var0
1
2
Atom unsafetraj(v20)
NegatedAtom unsafetraj(v20)
end_variable
begin_variable
var1
-1
2
Atom atconf(v0, #o20)
NegatedAtom atconf(v0, #o20)
end_variable
begin_variable
var2
-1
2
Atom atconf(v4, v5)
NegatedAtom atconf(v4, v5)
end_variable
begin_variable
var3
-1
2
Atom atconf(v0, #o21)
NegatedAtom atconf(v0, #o21)
end_variable
begin_variable
var4
-1
2
Atom atconf(v0, v17)
NegatedAtom atconf(v0, v17)
end_variable
begin_variable
var5
-1
2
Atom atconf(v0, v19)
NegatedAtom atconf(v0, v19)
end_variable
begin_variable
var6
-1
2
Atom atpose(v2, v3)
NegatedAtom atpose(v2, v3)
end_variable
begin_variable
var7
-1
2
Atom atgrasp(v0, v2, v16)
NegatedAtom atgrasp(v0, v2, v16)
end_variable
begin_variable
var8
-1
2
Atom handempty(v0)
NegatedAtom handempty(v0)
end_variable
begin_variable
var9
-1
2
Atom canmove(v0)
NegatedAtom canmove(v0)
end_variable
begin_variable
var10
-1
2
Atom atconf(v0, v1)
NegatedAtom atconf(v0, v1)
end_variable
begin_variable
var11
-1
2
Atom atpose(v2, v10)
NegatedAtom atpose(v2, v10)
end_variable
0
begin_state
0
1
0
1
1
1
0
1
0
0
0
1
end_state
begin_goal
2
10 0
11 0
end_goal
9
begin_operator
move v0 #o21 v17 #o23
0
3
0 3 0 1
0 4 -1 0
0 9 0 1
1
end_operator
begin_operator
move v0 v1 #o20 #o24
0
3
0 1 -1 0
0 10 0 1
0 9 0 1
1
end_operator
begin_operator
move v0 v17 v19 #o19
0
3
0 4 0 1
0 5 -1 0
0 9 0 1
1
end_operator
begin_operator
move v0 v19 v1 #o25
0
3
0 10 -1 0
0 5 0 1
0 9 0 1
1
end_operator
begin_operator
pick v0 v2 v10 v16 v19 v20
2
5 0
0 1
4
0 7 -1 0
0 11 0 1
0 9 -1 0
0 8 0 1
1
end_operator
begin_operator
pick v0 v2 v3 v16 v17 v18
1
4 0
4
0 7 -1 0
0 6 0 1
0 9 -1 0
0 8 0 1
1
end_operator
begin_operator
place v0 v2 v10 v16 v19 v20
2
5 0
0 1
4
0 7 0 1
0 11 -1 0
0 9 -1 0
0 8 -1 0
1
end_operator
begin_operator
place v0 v2 v3 v16 v17 v18
1
4 0
4
0 7 0 1
0 6 -1 0
0 9 -1 0
0 8 -1 0
1
end_operator
begin_operator
pull v0 #o20 #o21 v4 v5 v6 #o22
2
1 0
8 0
3
0 3 -1 0
0 2 0 1
0 9 -1 0
1
end_operator
1
begin_rule
1
2 1
0 0 1
end_rule
