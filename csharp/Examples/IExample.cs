namespace Examples
{
    public interface IExample
    {
        string Name { get; }
        string Description { get; }
        string Usage { get; }
        void Run(string[] args);
    }
}
