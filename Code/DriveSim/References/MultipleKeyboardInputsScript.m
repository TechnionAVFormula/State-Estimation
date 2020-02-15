NET.addAssembly('PresentationCore');
akey = System.Windows.Input.Key.A;  %use any key to get the enum type
keys = System.Enum.GetValues(akey.GetType);  %get all members of enumeration
keynames = cell(System.Enum.GetNames(akey.GetType))';
iskeyvalid = true(keys.Length, 1);
iskeydown = false(keys.Length, 1);
for keyidx = 1:keys.Length
   try
       iskeydown(keyidx) = System.Windows.Input.Keyboard.IsKeyDown(keys(keyidx));
   catch
       iskeyvalid(keyidx) = false;
   end
end

iskeydown(iskeyvalid) = arrayfun(@(keyidx) System.Windows.Input.Keyboard.IsKeyDown(keys(keyidx)), find(iskeyvalid));